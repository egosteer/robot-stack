#!/usr/bin/env python3
import argparse
import hashlib
import json
import math
import os
import threading
import subprocess
import sys
import time
import uuid
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import logging
from urllib.parse import parse_qs

import requests

try:
    import yaml
except ImportError:
    yaml = None

# Input mode: 'en' (English only, no translation) | 'cn' (Chinese only, auto-translate to English)
# | 'dev' (both editable, bidirectional translation). Set by main() from CLI flags.
MODE = 'en'

SCRIPT_DIR = Path(__file__).resolve().parent
HISTORY_FILE = SCRIPT_DIR / "command_history.json"
CONFIG_FILE = SCRIPT_DIR / "config.yaml"
AUDIO_DIR = SCRIPT_DIR / "audio"
AUDIO_CACHE_DIR = AUDIO_DIR / ".playback_cache"
AUDIO_CACHE_LOCK = threading.Lock()


def _coerce_str(value) -> str:
    if value is None:
        return ""
    return value if isinstance(value, str) else str(value)


# ==========================================
# Youdao Translation API (used by --cn / --dev)
# ==========================================
YOUDAO_URL = "https://openapi.youdao.com/api"


class TranslationError(RuntimeError):
    pass


def _load_youdao_keys():
    """Read Youdao app key/secret from config.yaml (env vars override)."""
    app_key = os.getenv("YOUDAO_APP_KEY", "")
    app_secret = os.getenv("YOUDAO_APP_SECRET", "")
    if (not app_key or not app_secret) and yaml is not None and CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            app_key = app_key or _coerce_str(cfg.get("youdao_app_key")).strip()
            app_secret = app_secret or _coerce_str(cfg.get("youdao_app_secret")).strip()
        except Exception:
            pass
    return app_key, app_secret


def _truncate_youdao_query(text: str) -> str:
    return text if len(text) <= 20 else f"{text[:10]}{len(text)}{text[-10:]}"


def request_youdao_translation(text: str, from_lang: str, to_lang: str, timeout: int = 60) -> str:
    text = text.strip()
    if not text:
        return ""
    app_key, app_secret = _load_youdao_keys()
    if not app_key or not app_secret:
        raise TranslationError("Youdao API not configured; set youdao_app_key / youdao_app_secret in config.yaml")

    salt = str(uuid.uuid4())
    curtime = str(int(time.time()))
    sign = hashlib.sha256(
        (app_key + _truncate_youdao_query(text) + salt + curtime + app_secret).encode("utf-8")
    ).hexdigest()
    data = {
        "q": text, "from": from_lang, "to": to_lang, "appKey": app_key,
        "salt": salt, "sign": sign, "signType": "v3", "curtime": curtime, "strict": "true",
    }
    try:
        resp = requests.post(YOUDAO_URL, data=data, timeout=timeout)
        resp.raise_for_status()
        payload = resp.json()
    except Exception as exc:
        raise TranslationError(f"Youdao request failed: {exc}") from exc
    if _coerce_str(payload.get("errorCode")) != "0":
        raise TranslationError(f"Youdao error code: {payload.get('errorCode')}")
    try:
        return _coerce_str(payload["translation"][0]).strip()
    except (KeyError, IndexError, TypeError) as exc:
        raise TranslationError("Youdao returned malformed data") from exc


# ==========================================
# Audio (mp3 lead-in cache)
# ==========================================
def _read_audio_lead_in_seconds() -> float:
    try:
        return max(0.0, float(os.getenv("AUDIO_LEAD_IN_SECONDS", "0.10")))
    except ValueError:
        return 0.10


AUDIO_LEAD_IN_SECONDS = _read_audio_lead_in_seconds()
MPEG1_LAYER3_BITRATES_KBPS = [None, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, None]
MPEG1_SAMPLE_RATES_HZ = [44100, 48000, 32000, None]


def _id3v2_tag_end(data: bytes) -> int:
    if len(data) < 10 or data[:3] != b"ID3":
        return 0
    size = 0
    for byte in data[6:10]:
        size = (size << 7) | (byte & 0x7F)
    return min(10 + size, len(data))


def _parse_mpeg1_layer3_frame(data: bytes, offset: int):
    if offset + 4 > len(data):
        return None
    header = int.from_bytes(data[offset:offset + 4], "big")
    if ((header >> 21) & 0x7FF) != 0x7FF:
        return None
    version, layer = (header >> 19) & 0x3, (header >> 17) & 0x3
    bitrate_index, sample_rate_index, padding = (header >> 12) & 0xF, (header >> 10) & 0x3, (header >> 9) & 0x1
    if version != 0x3 or layer != 0x1:
        return None
    bitrate_kbps = MPEG1_LAYER3_BITRATES_KBPS[bitrate_index]
    sample_rate_hz = MPEG1_SAMPLE_RATES_HZ[sample_rate_index]
    if bitrate_kbps is None or sample_rate_hz is None:
        return None
    frame_length = int((144000 * bitrate_kbps) // sample_rate_hz + padding)
    if frame_length <= 4 or offset + frame_length > len(data):
        return None
    return {"header": data[offset:offset + 4], "length": frame_length, "duration": 1152.0 / sample_rate_hz}


def _audio_cache_is_fresh(cache_path: Path, source_path: Path) -> bool:
    try:
        return (cache_path.exists()
                and cache_path.stat().st_mtime >= source_path.stat().st_mtime
                and cache_path.stat().st_size > source_path.stat().st_size)
    except OSError:
        return False


def _mp3_with_lead_in(file_path: Path) -> Path:
    if AUDIO_LEAD_IN_SECONDS <= 0:
        return file_path
    try:
        # Cache key includes the language subdir so same-named Chinese/English clips don't collide.
        stem_key = str(file_path.resolve().relative_to(AUDIO_DIR.resolve()).with_suffix('')).replace(os.sep, '_')
    except ValueError:
        stem_key = file_path.stem
    cache_path = AUDIO_CACHE_DIR / f"{stem_key}.lead{int(AUDIO_LEAD_IN_SECONDS * 1000)}ms.mp3"
    if _audio_cache_is_fresh(cache_path, file_path):
        return cache_path
    with AUDIO_CACHE_LOCK:
        if _audio_cache_is_fresh(cache_path, file_path):
            return cache_path
        try:
            data = file_path.read_bytes()
            first_frame_offset = _id3v2_tag_end(data)
            frame = _parse_mpeg1_layer3_frame(data, first_frame_offset)
            if frame is None:
                return file_path
            first_frame_end = first_frame_offset + frame["length"]
            first_frame = data[first_frame_offset:first_frame_end]
            insert_at = first_frame_end if (b"Xing" in first_frame or b"Info" in first_frame) else first_frame_offset
            lead_in_frames = max(1, math.ceil(AUDIO_LEAD_IN_SECONDS / frame["duration"]))
            silence_frame = frame["header"] + (b"\x00" * (frame["length"] - 4))
            AUDIO_CACHE_DIR.mkdir(parents=True, exist_ok=True)
            temp_path = cache_path.with_name(f"{cache_path.name}.{os.getpid()}.tmp")
            temp_path.write_bytes(data[:insert_at] + silence_frame * lead_in_frames + data[insert_at:])
            temp_path.replace(cache_path)
            return cache_path
        except OSError as exc:
            logger.warning(f"Failed to generate lead-in audio cache; playing original: {exc}")
            return file_path


def get_audio_playback_path(file_path: Path) -> Path:
    return _mp3_with_lead_in(file_path) if file_path.suffix.lower() == ".mp3" else file_path


# ==========================================
# State and history
# ==========================================
class GlobalState:
    def __init__(self):
        self.is_robot_waiting = False
        self.current_command = None     # English instruction sent to the robot
        self.command_event = threading.Event()
        self.server_ready = False
        self.history = self._load_history()

    def _load_history(self):
        if HISTORY_FILE.exists():
            try:
                with open(HISTORY_FILE, 'r', encoding='utf-8') as f:
                    return self._normalize_history(json.load(f))
            except Exception:
                return []
        return []

    def _normalize_item(self, item):
        if isinstance(item, dict):
            english = _coerce_str(item.get("english") or item.get("instruction") or item.get("cmd")).strip()
            chinese = _coerce_str(item.get("chinese") or item.get("zh")).strip()
        else:
            english, chinese = _coerce_str(item).strip(), ""
        if not english and not chinese:
            return None
        return {"english": english, "chinese": chinese}

    def _normalize_history(self, history):
        if not isinstance(history, list):
            return []
        out = []
        for item in history:
            entry = self._normalize_item(item)
            if entry:
                out.append(entry)
        return out

    def save_command(self, english, chinese=""):
        entry = self._normalize_item({"english": english, "chinese": chinese})
        if not entry or not entry["english"]:
            return
        self.history = [h for h in self._normalize_history(self.history) if h["english"] != entry["english"]]
        self.history.insert(0, entry)
        self.history = self.history[:50]
        try:
            with open(HISTORY_FILE, 'w', encoding='utf-8') as f:
                json.dump(self.history, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"Failed to save history: {e}")


state = GlobalState()

# ==========================================
# Logging
# ==========================================
class CleanLoggingHandler(logging.StreamHandler):
    def emit(self, record):
        try:
            sys.stdout.write('\r\033[K' + self.format(record) + '\n')
            sys.stdout.flush()
        except Exception:
            self.handleError(record)


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
for h in logger.handlers[:]:
    logger.removeHandler(h)
_handler = CleanLoggingHandler()
_handler.setFormatter(logging.Formatter('\033[94m%(asctime)s\033[0m - %(message)s'))
logger.addHandler(_handler)

# ==========================================
# Web page
# ==========================================
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Stack Instruction Terminal</title>
    <style>
        :root { --blue:#2f80ed; --blue-dark:#1f64c8; --green:#219653; --ink:#233142; --muted:#6b7c8f;
                --line:#d7dee8; --panel:#fff; --page:#f4f7f9; --soft-gray:#edf1f5; --danger:#c0392b; }
        body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; max-width:980px; margin:32px auto;
               padding:20px; background:var(--page); color:var(--ink); transition:background .4s; }
        .card { background:var(--panel); padding:28px; border-radius:8px; box-shadow:0 10px 30px rgba(22,35,48,.08); text-align:center; }
        .status-idle{background:#f0f2f5;} .status-active{background:#e3f2fd;}
        h1 { margin:0 0 8px; color:#2c3e50; font-size:24px; }
        .mode-hint { font-size:12px; color:var(--muted); margin:0 0 18px; }
        .mode-hint code { font-family:ui-monospace,Menlo,monospace; background:var(--soft-gray); padding:1px 5px; border-radius:4px; }
        .grid { display:grid; gap:14px; align-items:stretch; margin-bottom:12px; }
        .grid.dev { grid-template-columns:minmax(0,1fr) 70px minmax(0,1fr); }
        .grid.single { grid-template-columns:minmax(0,1fr); }
        .input-group { display:flex; flex-direction:column; gap:8px; text-align:left; }
        .input-label { color:var(--muted); font-size:13px; font-weight:700; padding-left:2px; }
        textarea { width:100%; height:150px; min-height:150px; max-height:260px; padding:14px; border:2px solid var(--line);
                   border-radius:8px; font-size:16px; box-sizing:border-box; outline:none; resize:vertical; font-family:inherit;
                   line-height:1.5; transition:border .3s,box-shadow .3s; }
        textarea:focus { border-color:var(--blue); box-shadow:0 0 0 3px rgba(47,128,237,.14); }
        textarea:disabled { background:#f9f9f9; cursor:not-allowed; }
        .translate-controls { display:flex; flex-direction:column; justify-content:center; gap:10px; padding-top:29px; }
        button { width:100%; padding:16px; background:var(--blue); color:#fff; border:none; border-radius:8px; font-size:18px;
                 cursor:pointer; font-weight:bold; transition:all .3s; }
        button:hover:not(:disabled){background:var(--blue-dark);} button:disabled{background:#bdc3c7;cursor:not-allowed;}
        .arrow-btn { height:52px; padding:0; background:var(--green); font-size:26px; line-height:1; }
        .arrow-btn:hover:not(:disabled){background:#1b7f45;}
        #indicator { font-weight:bold; margin-bottom:20px; padding:12px; border-radius:10px; font-size:14px; }
        .idle-msg { color:#7f8c8d; background:#e9ecef; border:1px solid #dee2e6; }
        .active-msg { color:#2980b9; background:#d1ecf1; border:1px solid #3498db; }
        .status-line { min-height:22px; margin:6px 0 16px; color:var(--muted); font-size:14px; text-align:left; }
        .status-line.ok{color:var(--green);} .status-line.error{color:var(--danger);} .status-line.working{color:var(--blue);}
        .history-section { margin-top:30px; text-align:left; }
        .history-title { font-size:13px; color:#95a5a6; margin-bottom:10px; font-weight:bold; text-transform:uppercase; padding-left:5px; }
        .history-list { display:flex; flex-direction:column; gap:8px; max-height:350px; overflow-y:auto; padding-right:5px; }
        .history-item { background:#fff; padding:12px 15px; border-radius:8px; border:1px solid #eee; font-size:14px; color:#444;
                        cursor:pointer; transition:all .2s; white-space:pre-wrap; word-break:break-word; }
        .history-item:hover { background:#f8f9fa; border-color:#3498db; color:#3498db; transform:translateX(5px); }
        .history-english { color:#2c3e50; line-height:1.45; }
        .history-chinese { margin-top:6px; color:#6b7c8f; line-height:1.45; }
    </style>
</head>
<body class="status-idle">
    <div class="card">
        <div id="indicator" class="idle-msg"></div>
        <h1 id="pageTitle"></h1>
        <div id="modeHint" class="mode-hint"></div>
        <form id="cmdForm">
            <div id="grid" class="grid single">
                <div class="input-group" id="englishGroup">
                    <label class="input-label" id="englishLabel" for="englishInstruction">English</label>
                    <textarea id="englishInstruction" placeholder="..." disabled autocomplete="off" lang="en"></textarea>
                </div>
                <div class="translate-controls" id="translateControls" style="display:none">
                    <button type="button" class="arrow-btn" id="toChineseBtn" title="English to Chinese" disabled>&rarr;</button>
                    <button type="button" class="arrow-btn" id="toEnglishBtn" title="Chinese to English" disabled>&larr;</button>
                </div>
                <div class="input-group" id="chineseGroup" style="display:none">
                    <label class="input-label" id="chineseLabel" for="chineseInstruction">Chinese</label>
                    <textarea id="chineseInstruction" placeholder="..." disabled autocomplete="off" lang="zh-CN"></textarea>
                </div>
            </div>
            <div id="statusLine" class="status-line"></div>
            <button type="submit" id="submitBtn" disabled></button>
        </form>
        <div class="history-section">
            <div class="history-title" id="historyTitle"></div>
            <div id="historyList" class="history-list"></div>
        </div>
    </div>
    <script>
        const MODE = "__MODE__";  // 'en' | 'cn' | 'dev'
        const T = {
            en: {
                title:"Robot Stack Instruction Terminal", idle:"Inference program is not requesting input; waiting...",
                active:"Inference program is requesting input; please respond", enLabel:"English", zhLabel:"Chinese",
                submit:"Confirm and Send Instruction", sent:"Sent", historyTitle:"Recently Used Instructions",
                phType:"Type the instruction...", phWait:"Waiting for a request...",
                hintEn:"Type an English instruction and press Enter.",
                hintCn:"<strong>Chinese input mode</strong>: type Chinese; Enter auto-translates to English and sends.",
                hintDev:"<strong>Developer mode</strong>: edit either box; Enter translates from the last-edited side and sends. Center buttons translate manually.",
                translating:"Translating...", translated:"Translation complete", translateFail:"Translation failed",
                enterToTranslate:"Enter text to translate first", emptyInstr:"Instruction is empty", enterCn:"Enter a Chinese instruction",
                enterInstr:"Enter an instruction", instrSent:"Instruction sent", sendFail:"Send failed",
                emptyEn:"English instruction is empty", noEnglish:"(no English)"
            },
            cn: {
                title:"机器人指令终端", idle:"推理程序暂未请求输入，等待中…",
                active:"推理程序正在请求输入，请输入指令", enLabel:"英文", zhLabel:"中文",
                submit:"确认并发送指令", sent:"已发送", historyTitle:"最近使用的指令",
                phType:"请输入指令…", phWait:"等待请求…",
                hintEn:"输入英文指令后回车。",
                hintCn:"<strong>中文输入模式</strong>：输入中文，回车后自动翻译为英文并发送。",
                hintDev:"",
                translating:"翻译中…", translated:"翻译完成", translateFail:"翻译失败",
                enterToTranslate:"请先输入要翻译的文本", emptyInstr:"指令为空", enterCn:"请输入中文指令",
                enterInstr:"请输入指令", instrSent:"指令已发送", sendFail:"发送失败",
                emptyEn:"英文指令为空", noEnglish:"（无英文）"
            }
        };
        const L = T[MODE==='cn' ? 'cn' : 'en'];
        const el = id => document.getElementById(id);
        const form=el('cmdForm'), englishInput=el('englishInstruction'), chineseInput=el('chineseInstruction'),
              grid=el('grid'), englishGroup=el('englishGroup'), chineseGroup=el('chineseGroup'),
              translateControls=el('translateControls'), toChineseBtn=el('toChineseBtn'), toEnglishBtn=el('toEnglishBtn'),
              btn=el('submitBtn'), indicator=el('indicator'), statusLine=el('statusLine'),
              historyList=el('historyList'), modeHint=el('modeHint'), body=document.body;

        el('pageTitle').textContent=L.title; el('englishLabel').textContent=L.enLabel;
        el('chineseLabel').textContent=L.zhLabel; el('historyTitle').textContent=L.historyTitle;
        btn.textContent=L.submit; indicator.textContent=L.idle;

        let lastHistoryJSON="", waiting=false, busy=false, translating=false, lastModified='zh';
        let lastTranslated={en:"",zh:""};

        // Layout per mode
        if (MODE==='dev') {
            grid.className='grid dev'; chineseGroup.style.display=''; translateControls.style.display='';
            modeHint.innerHTML=L.hintDev;
        } else if (MODE==='cn') {
            grid.className='grid single'; englishGroup.style.display='none'; chineseGroup.style.display='';
            modeHint.innerHTML=L.hintCn;
        } else {
            modeHint.innerHTML=L.hintEn;
        }

        function setStatus(m,t=""){ statusLine.textContent=m; statusLine.className="status-line"+(t?" "+t:""); }
        const inputs = MODE==='dev' ? [englishInput,chineseInput] : (MODE==='cn'?[chineseInput]:[englishInput]);
        inputs.forEach(inp=>inp.addEventListener('keydown',e=>{ if(e.key==='Enter'&&!e.shiftKey){e.preventDefault(); if(!btn.disabled) form.requestSubmit();}}));
        if (MODE==='dev'){ englishInput.addEventListener('input',()=>lastModified='en'); chineseInput.addEventListener('input',()=>lastModified='zh'); }

        function refresh(){
            const locked = !waiting || busy || translating;
            btn.disabled=locked;
            englishInput.disabled = (MODE==='en'||MODE==='dev') ? locked : true;
            chineseInput.disabled = (MODE==='cn'||MODE==='dev') ? locked : true;
            if (MODE==='dev'){ toChineseBtn.disabled=locked; toEnglishBtn.disabled=locked; }
            const ph = waiting ? L.phType : L.phWait;
            englishInput.placeholder=ph; chineseInput.placeholder=ph;
        }

        async function translateText(src, dst, fromLang, toLang){
            const text=src.value.trim();
            if(!text){ setStatus(L.enterToTranslate,"error"); return false; }
            translating=true; setStatus(L.translating,"working"); refresh();
            let ok=false;
            try{
                const r=await fetch('/translate',{method:'POST',body:new URLSearchParams({text,from:fromLang,to:toLang})});
                const d=await r.json().catch(()=>({}));
                if(!r.ok) throw new Error(d.error||L.translateFail);
                dst.value=d.translation||""; setStatus(L.translated,"ok");
                const sk=fromLang==='zh-CHS'?'zh':'en'; lastTranslated[sk]=text; lastTranslated[sk==='zh'?'en':'zh']=(d.translation||"").trim();
                ok=true;
            }catch(e){ setStatus(e.message||L.translateFail,"error"); }
            finally{ translating=false; refresh(); }
            return ok;
        }
        toChineseBtn.addEventListener('click',()=>{ if(MODE==='dev'){lastModified='en'; translateText(englishInput,chineseInput,'en','zh-CHS');} });
        toEnglishBtn.addEventListener('click',()=>{ if(MODE==='dev'){lastModified='zh'; translateText(chineseInput,englishInput,'zh-CHS','en');} });

        function updateHistory(history){
            const j=JSON.stringify(history); if(j===lastHistoryJSON) return; lastHistoryJSON=j;
            historyList.innerHTML='';
            history.forEach(item=>{
                const en=String((item&&item.english)||item||"").trim(), zh=String((item&&item.chinese)||"").trim();
                if(!en&&!zh) return;
                const div=document.createElement('div'); div.className='history-item';
                const e1=document.createElement('div'); e1.className='history-english'; e1.textContent=en||L.noEnglish; div.appendChild(e1);
                if(zh){ const c1=document.createElement('div'); c1.className='history-chinese'; c1.textContent=zh; div.appendChild(c1); }
                div.onclick=()=>{ if(!waiting||busy||translating) return; englishInput.value=en; chineseInput.value=zh;
                    lastTranslated.en=en; lastTranslated.zh=zh; };
                historyList.appendChild(div);
            });
        }

        function setWaiting(w){
            const was=waiting; waiting=Boolean(w);
            if(waiting){ indicator.textContent=L.active; indicator.className="active-msg";
                         body.classList.remove('status-idle'); body.classList.add('status-active'); }
            else { busy=false; indicator.textContent=L.idle; indicator.className="idle-msg";
                   body.classList.remove('status-active'); body.classList.add('status-idle'); setStatus(""); }
            refresh();
            if(waiting&&!was) (MODE==='cn'?chineseInput:englishInput).focus();
        }

        async function poll(){ try{ const d=await (await fetch('/status')).json(); setWaiting(d.waiting); if(d.history) updateHistory(d.history);}catch(e){} }
        setInterval(poll,800);

        async function ensureEnglish(){
            if(MODE==='en') return englishInput.value.trim()?true:(setStatus(L.emptyInstr,"error"),false);
            if(MODE==='cn'){
                const zh=chineseInput.value.trim();
                if(!zh){ setStatus(L.enterCn,"error"); return false; }
                if(zh!==lastTranslated.zh){ if(!await translateText(chineseInput,englishInput,'zh-CHS','en')) return false; }
                return Boolean(englishInput.value.trim());
            }
            // dev
            let sk=lastModified, src=sk==='zh'?chineseInput:englishInput, dst=sk==='zh'?englishInput:chineseInput;
            if(!src.value.trim()&&dst.value.trim()){ sk=sk==='zh'?'en':'zh'; src=sk==='zh'?chineseInput:englishInput; dst=sk==='zh'?englishInput:chineseInput; }
            if(!src.value.trim()){ setStatus(L.enterInstr,"error"); return false; }
            if(src.value.trim()!==lastTranslated[sk]){ if(!await translateText(src,dst,sk==='zh'?'zh-CHS':'en',sk==='zh'?'en':'zh-CHS')) return false; }
            return Boolean(englishInput.value.trim());
        }

        form.onsubmit=async e=>{
            e.preventDefault(); if(!waiting||busy) return;
            if(!await ensureEnglish()) return;
            const en=englishInput.value.trim();
            if(!en){ setStatus(L.emptyEn,"error"); return; }
            busy=true; refresh(); const orig=L.submit;
            try{
                const r=await fetch('/web_submit',{method:'POST',body:new URLSearchParams({instruction:en, chinese:chineseInput.value.trim()})});
                const d=await r.json().catch(()=>({}));
                if(!r.ok||d.accepted===false) throw new Error(d.error||L.sendFail);
                btn.textContent=L.sent; setStatus(L.instrSent,"ok"); setTimeout(()=>btn.textContent=orig,1000);
            }catch(e){ busy=false; btn.textContent=orig; setStatus(e.message||L.sendFail,"error"); refresh(); }
        };
        refresh(); poll();
    </script>
</body>
</html>
"""


# ==========================================
# Server
# ==========================================
class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class InteractionHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path == '/':
            self._send_html()
        elif self.path == '/status':
            self._send_json({"waiting": state.is_robot_waiting, "history": state.history})
        elif self.path == '/get_input':
            self._handle_robot_request()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/web_submit':
            self._handle_web_submit()
        elif self.path == '/translate':
            self._handle_translate()
        elif self.path.startswith('/play/'):
            self._handle_play(self.path[6:])
        else:
            self.send_error(404)

    def _handle_robot_request(self):
        logger.info("📡 Inference program requested input")
        state.command_event.clear()
        state.is_robot_waiting = True
        state.command_event.wait(timeout=None)
        if state.current_command:
            self._send_json({"instruction": state.current_command})
            logger.info(f"✅ Instruction delivered: '{state.current_command}'")
        state.is_robot_waiting = False
        state.current_command = None

    def _handle_web_submit(self):
        params = self._read_form()
        instr = params.get('instruction', [''])[0].strip()
        chinese = params.get('chinese', [''])[0].strip()
        if not instr:
            self._send_json({"accepted": False, "error": "Instruction cannot be empty"}, 400)
            return
        if not state.is_robot_waiting:
            self._send_json({"accepted": False, "error": "Inference program is not requesting input"}, 409)
            return
        state.current_command = instr
        state.save_command(instr, chinese)
        state.command_event.set()
        self._send_json({"accepted": True})

    def _handle_translate(self):
        params = self._read_form()
        text = params.get('text', [''])[0].strip()
        from_lang = params.get('from', [''])[0].strip()
        to_lang = params.get('to', [''])[0].strip()
        if not text:
            self._send_json({"error": "Text to translate cannot be empty"}, 400)
            return
        if (from_lang, to_lang) not in {('en', 'zh-CHS'), ('zh-CHS', 'en')}:
            self._send_json({"error": "Unsupported translation direction"}, 400)
            return
        try:
            self._send_json({"translation": request_youdao_translation(text, from_lang, to_lang)})
        except TranslationError as exc:
            logger.warning(f"⚠️ Translation failed: {exc}")
            self._send_json({"error": str(exc)}, 502)

    def _handle_play(self, name):
        # name is "<Language>/<clip>" (e.g. English/start) or a bare "<clip>".
        name = name.strip('/')
        if not name or '..' in name.split('/'):
            self.send_response(400); self.end_headers(); return
        for ext in ['.mp3', '.wav']:
            file_path = AUDIO_DIR / f"{name}{ext}"
            if file_path.exists():
                subprocess.Popen(['mpg123', '-q', str(get_audio_playback_path(file_path))],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                break
        self.send_response(200); self.end_headers()

    def _read_form(self):
        n = int(self.headers.get('Content-Length', 0) or 0)
        return parse_qs(self.rfile.read(n).decode('utf-8')) if n > 0 else {}

    def _send_json(self, data, status=200):
        body = json.dumps(data).encode('utf-8')
        self.send_response(status); self.send_header('Content-Type', 'application/json'); self.end_headers()
        self.wfile.write(body)

    def _send_html(self):
        self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.end_headers()
        self.wfile.write(HTML_PAGE.replace("__MODE__", MODE).encode('utf-8'))


def terminal_status_thread():
    while not state.server_ready:
        time.sleep(0.1)
    while True:
        msg = "🎯 requesting input; use the web client..." if state.is_robot_waiting else "☁️ not requesting input; waiting..."
        sys.stdout.write("\r\033[K" + msg); sys.stdout.flush()
        time.sleep(0.5)


def main():
    global MODE
    parser = argparse.ArgumentParser(description="Robot Stack instruction terminal")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--cn", action="store_true", help="Chinese UI; type Chinese, auto-translate to English")
    group.add_argument("--dev", action="store_true", help="Developer mode: edit both, bidirectional translation")
    args = parser.parse_args()
    MODE = 'cn' if args.cn else ('dev' if args.dev else 'en')

    threading.Thread(target=terminal_status_thread, daemon=True).start()
    port = 8081
    label = {'en': 'English only', 'cn': 'Chinese UI + input (auto-translate)', 'dev': 'Developer (bilingual)'}[MODE]
    print("\n" + "=" * 55)
    print("🚀 Robot Stack instruction terminal ready")
    print(f"🧭 Input mode: {label}")
    print(f"🔗 Control panel: http://localhost:{port}")
    print("=" * 55 + "\n")
    time.sleep(0.2)
    state.server_ready = True
    try:
        ThreadingHTTPServer(('0.0.0.0', port), InteractionHandler).serve_forever()
    except KeyboardInterrupt:
        print("\nService stopped")
        sys.exit(0)


if __name__ == '__main__':
    main()
