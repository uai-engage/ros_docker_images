# Quick Start - Ubuntu Desktop Container

Get up and running in **3 simple steps**.

## Step 1: Start Container (5 minutes)

```bash
cd docker
cp .env.example .env
docker compose build    # First time: ~5-10 minutes
docker compose up -d
```

## Step 2: Access Desktop

Open browser: **http://localhost:6901/vnc.html**

Password: `password`

You'll see Ubuntu Desktop!

## Step 3: Install Everything (1-1.5 hours)

In the VNC desktop, open Terminal and run:

```bash
~/install-all.sh
```

Go get coffee ☕ - installation is automated!

---

## That's It!

After installation completes, open terminals and run:

```bash
# Terminal 1
gz-default

# Terminal 2
px4-sitl

# Terminal 3
microros

# Terminal 4
px4-topics
```

## Connect QGroundControl

On your **host machine**:
- Type: **TCP**
- Server: **localhost**
- Port: **5760**

---

## Full Documentation

- `README.md` - Complete Docker container guide
- `~/installation-docs/USAGE-GUIDE.md` - Usage after installation

## Troubleshooting

**Can't access VNC?**
```bash
docker compose logs
docker compose restart
```

**Installation failed?**
```bash
# Inside VNC terminal:
cat ~/installation.log
```

**Need more resources?**
Edit `.env`:
```env
MEMORY_LIMIT=20G
CPU_LIMIT=12
```

Then restart:
```bash
docker compose restart
```
