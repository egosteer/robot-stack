# Stable serial device names (udev)

These rules give each USB serial device a fixed name under `/dev` (e.g. `/dev/glove_left`),
matched by USB vendor/product id, and for the two hands (one shared `1a86:55d5` quad-serial
adapter) also by interface number. The name **follows the device, not the port**. Swapping a
same-model glove or hand, moving it to another port or hub, or rebooting leaves the name
unchanged, with no edits needed.

## Install (on the host, once)

```bash
cd assets/udev_rules
./install.sh
```

`create_container.sh` mounts the host `/dev` into the container, so these names are visible
inside it too. The glove / hand launch files default to them (`/dev/glove_left`, `/dev/hand_left`,
`/dev/glove_right`, `/dev/hand_right`).

## Changing a mapping

Edit `99-robot-serial.rules` (e.g. if a hand moves to a different adapter interface) and re-run
`./install.sh`, which overwrites the installed rule and reloads udev. Re-plug the device if the
symlink does not refresh immediately.