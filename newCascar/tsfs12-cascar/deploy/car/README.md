# Car container setup

The Raspberry Pi is a static Docker host. Application code is built into a
versioned ARM64 image and is never mounted from the host. The laptop transfers
the image, Compose configuration, and car environment file directly over SSH.
The Pi never pulls images from a registry, including at boot or when enabling
LiDAR. Neither device needs internet access during deployment.

The runtime uses two containers built from that same image:

- `cascar` always runs the Arduino control and IMU nodes;
- `lidar` runs only the LiDAR node and is optional.

This keeps the image identical on every car while allowing LiDAR to be changed
per car without rebuilding the image.

## Quick start

Run these commands from the repository root. Configure the image once; the
helper reads `CASCAR_IMAGE` from the same file for pulling, building, and deploying.

### 1. Configure your car

```bash
cp deploy/car/car.env.example deploy/car/turtle.env
```

Edit `turtle.env`: set the car name, LiDAR model, ROS domain, and image. For a
published image, use its full registry name and version:

```dotenv
CASCAR_IMAGE=registry.example/cascar-car:abc1234
```

### 2. Get the image onto the laptop (online)

For a published image:

```bash
# Only needed for a private registry:
docker login registry.example

tools/car pull deploy/car/turtle.env
```

Or, to build from this checkout, set `CASCAR_IMAGE=cascar-car:dev` and run:

```bash
tools/car build deploy/car/turtle.env
```

Both commands select ARM64 automatically. Finish this step before leaving
internet access. Building requires the one-time laptop setup below.

### 3. Deploy to the car (hotspot only)

```bash
tools/car deploy pi@turtle.local deploy/car/turtle.env
```

Use the Pi's IP address or an SSH config alias if `.local` discovery is unavailable.
The helper copies the image and configuration over SSH, loads the image, and
restarts the service. It allows sudo to prompt for a password on the Pi.
Neither device needs internet access for this step.

```bash
tools/car status pi@turtle.local
tools/car logs pi@turtle.local
```

For updates or rollbacks, change `CASCAR_IMAGE`, pull or build it onto the
laptop, and deploy again. If it is already on the laptop, just deploy.

## One-time setup

### Raspberry Pi

The Pi needs:

- a 64-bit OS compatible with ROS 2 Jazzy images;
- Docker Engine with the Compose plugin;
- SSH access from the laptop;
- I2C enabled as `/dev/i2c-1`;
- [`udev/cascar.rules`](udev/cascar.rules) and
  [`udev/i2c.rules`](udev/i2c.rules) installed under `/etc/udev/rules.d`,
  creating `/dev/arduino` and `/dev/rplidar`.

Prepare these beforehand. Deployment installs the application configuration
and startup service; it does not install Docker or change the base OS.
The Pi does not need registry credentials.

### Laptop

Install Docker Engine/Desktop with Compose. Building also requires buildx and
an ARM64-capable builder. On a Linux laptop with a different CPU architecture,
set up emulation once while online:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx inspect --bootstrap
```

The inspection output must include `linux/arm64` under `Platforms`. Docker
Desktop normally provides emulation already. A native ARM64 builder also works.
Pulling a prebuilt image does not require emulation.

## Optional registry sharing

To build and publish for others, set `CASCAR_IMAGE` to your registry image name
in the environment file, log in to the registry on the laptop, and run:

```bash
tools/car publish deploy/car/turtle.env
```

Publishing pushes the build to the registry. To deploy that image yourself,
use `pull` and then `deploy` as shown above.

`pull`, `build`, and `publish` also accept an image name directly:

```bash
tools/car pull registry.example/cascar-car:abc1234
tools/car build cascar-car:dev
tools/car publish registry.example/cascar-car:abc1234
```

All three default to `linux/arm64`. Set `CASCAR_PLATFORM` only for a car with
another supported 64-bit architecture. Use `tools/car --help` for command help.

## Deployment details

- Deployment checks the image's CPU architecture before copying it.
- The image is compressed for transfer. Allow space on both devices for the
  archive, and on the Pi for the loaded image.
- Transfer or image-load failures leave the running service alone. Retry the
  same deployment command.
- Configuration is resolved on the laptop. Change hardware settings in the
  laptop environment file and redeploy.
- The service starts automatically on reboot. Registry pulls are disabled,
  including when enabling LiDAR; missing local images cause an error.
- Temporary files are cleaned up on exit. An abrupt laptop shutdown may leave
  `/tmp/cascar-deploy.*` on the Pi; remove it once no deployment is running.
- Old Docker images are retained for reuse; the helper does not prune them.

## Enable or disable LiDAR from the laptop

First deploy this version of the Compose configuration and image to the car.
After that, LiDAR can be changed independently.

Enable it:

```bash
tools/car lidar pi@turtle.local on
```

Disable it:

```bash
tools/car lidar pi@turtle.local off
```

The command updates `COMPOSE_PROFILES` directly in `/etc/cascar/car.env` on the
Pi, so the setting survives a reboot. A later `tools/car deploy` overwrites the
complete on-car environment with the supplied laptop environment file. Turning
LiDAR off stops and removes only the LiDAR container; the core car controller
is not restarted.

To select the hardware model, set `CASCAR_LIDAR_MODEL=a2m8` or
`CASCAR_LIDAR_MODEL=a3` in the car's environment file. Changing the model
requires deploying the changed environment file.

If a car should start without LiDAR on its very first deployment, set this in
its environment file before deploying:

```dotenv
COMPOSE_PROFILES=
```
