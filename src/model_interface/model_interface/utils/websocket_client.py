# Adapted from openpi-client (https://github.com/Physical-Intelligence/openpi),
# Copyright (c) Physical Intelligence, licensed under the Apache License, Version 2.0.

import logging
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from . import msgpack_numpy
import websockets.sync.client


class WebsocketClientPolicy:
    """Implements the Policy interface by communicating with a server over websocket.

    See WebsocketPolicyServer for a corresponding server implementation.
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: Optional[int] = None,
        api_key: Optional[str] = None,
        image_compression: str = "none",
        jpeg_quality: int = 80,
    ) -> None:
        if host.startswith("ws"):
            self._uri = host
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"
        self._packer = msgpack_numpy.Packer()
        self._api_key = api_key
        self._image_compression = image_compression
        self._jpeg_quality = int(jpeg_quality)
        self._validate_image_compression_config()
        self._ws, self._server_metadata = self._wait_for_server()

    def get_server_metadata(self) -> Dict:
        return self._server_metadata

    def _wait_for_server(self) -> Tuple[websockets.sync.client.ClientConnection, Dict]:
        logging.info(f"Waiting for server at {self._uri}...")
        while True:
            try:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    additional_headers=headers,
                    ping_interval=60,
                    ping_timeout=90,
                    close_timeout=10
                )
                metadata = msgpack_numpy.unpackb(conn.recv())
                return conn, metadata
            except ConnectionRefusedError:
                logging.info("Still waiting for server...")
                time.sleep(5)

    def infer(self, obs: Dict) -> Dict:  # noqa: UP006
        data = self._packer.pack(self._prepare_obs_for_transport(obs))
        self._ws.send(data)
        response = self._ws.recv()
        if isinstance(response, str):
            # we're expecting bytes; if the server sends a string, it's an error.
            raise RuntimeError(f"Error in inference server:\n{response}")
        return msgpack_numpy.unpackb(response)

    def reset(self) -> None:
        pass

    def _validate_image_compression_config(self) -> None:
        if self._image_compression not in ("none", "jpeg"):
            raise ValueError("image_compression must be 'none' or 'jpeg'")
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")

    def _prepare_obs_for_transport(self, obs: Dict) -> Dict:
        if self._image_compression == "none":
            return obs

        transport_obs = dict(obs)
        if "image" in transport_obs:
            transport_obs["image"] = self._encode_image_field_as_jpeg(transport_obs["image"])
            transport_obs["image_compression"] = {
                "format": "jpeg",
                "quality": self._jpeg_quality,
                "color_order": "rgb",
                "field": "image",
            }
        return transport_obs

    def _encode_image_field_as_jpeg(self, image_field):
        if isinstance(image_field, dict):
            return {
                camera_name: self._encode_rgb_sequence_as_jpeg(image_sequence)
                for camera_name, image_sequence in image_field.items()
            }
        return self._encode_rgb_sequence_as_jpeg(image_field)

    def _encode_rgb_sequence_as_jpeg(self, image_sequence) -> Dict:
        arr = np.asarray(image_sequence)
        original_shape = tuple(arr.shape)
        if arr.ndim == 3:
            arr = arr[np.newaxis, ...]
        if arr.ndim != 4 or arr.shape[-1] != 3:
            raise ValueError(
                "JPEG image compression expects RGB images with shape "
                "(T, H, W, 3) or (H, W, 3)"
            )
        if arr.dtype != np.uint8:
            raise ValueError(f"JPEG image compression expects uint8 RGB images, got {arr.dtype}")

        frames = []
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        for frame in arr:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode(".jpg", frame_bgr, params)
            if not ok:
                raise RuntimeError("Failed to JPEG-encode RGB image frame")
            frames.append(encoded.tobytes())

        return {
            "__image_encoding__": "jpeg_sequence",
            "format": "jpeg",
            "quality": self._jpeg_quality,
            "shape": original_shape,
            "dtype": str(arr.dtype),
            "color_order": "rgb",
            "frames": frames,
            "raw_nbytes": int(arr.nbytes),
            "encoded_nbytes": int(sum(len(frame) for frame in frames)),
        }
