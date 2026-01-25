# ✅ **UNO Q Docker Storage Migration Checklist (Final Version)**  
*A complete, safe, repeatable guide based on your real-world migration.*

---

## **1. Create the new Docker storage directory**
Use the partition with free space:

```
sudo mkdir -p /home/arduino/docker-data
```

---

## **2. Stop Docker completely**
Docker has two systemd units. Stop both:

```
sudo systemctl stop docker.service
sudo systemctl stop docker.socket
```

Verify both are inactive:

```
systemctl status docker.service
systemctl status docker.socket
```

Both should show **inactive (dead)**.

---

## **3. Edit Docker’s daemon.json**
Open:

```
sudo nano /etc/docker/daemon.json
```

Add the new key inside the top-level JSON object:

```json
{
    "log-driver": "json-file",
    "log-opts": {
        "max-size": "10m",
        "max-file": "2"
    },
    "data-root": "/home/arduino/docker-data"
}
```

Save and exit.

---

## **4. (Optional but recommended) Migrate old Docker data**
If you want to keep your old images and containers:

### If rsync is installed:
```
sudo rsync -aP /var/lib/docker/ /home/arduino/docker-data/
```

### If rsync is NOT installed (UNO Q default):
Use cp -a:

```
sudo cp -a /var/lib/docker/* /home/arduino/docker-data/
```

This preserves permissions, symlinks, and structure.

---

## **5. Disable the old Docker directory**
Rename it so Docker won’t use it:

```
sudo mv /var/lib/docker /var/lib/docker.bak
```

This keeps a safe backup.

---

## **6. Restart Docker cleanly**
Start the socket first, then the daemon:

```
sudo systemctl start docker.socket
sudo systemctl start docker.service
```

---

## **7. Verify Docker is using the new storage**
Run:

```
docker info | grep "Docker Root Dir"
```

You should see:

```
Docker Root Dir: /home/arduino/docker-data
```

This confirms the migration succeeded.

---

## **8. Confirm Docker is writing to the new directory**
Check:

```
ls -lh /home/arduino/docker-data
```

You should see directories like:

- `overlay2/`
- `image/`
- `containers/`
- `volumes/`
- `buildkit/`

If not, pull a tiny image:

```
docker pull hello-world
```

Then check again.

---

## **9. (Optional) Reclaim space by deleting the backup**
Once you’re confident everything works:

```
sudo rm -rf /var/lib/docker.bak
```

This frees up space on the root partition.

---

## **10. Verify free space**
Check that `/` is no longer full:

```
df -h /
```

And confirm `/home/arduino` has plenty of room:

```
df -h /home/arduino
```

---

# 🎉 You now have a fully migrated, stable Docker environment  
This checklist reflects **exactly** what you encountered, including:

- Docker socket activation  
- Missing rsync  
- Root partition at 100%  
- Successful migration to `/home/arduino/docker-data`  
- Verification steps  

If you want, I can also generate:

- a **cleanup checklist** for keeping Docker lean  
- a **ROS build checklist** for your sonar + SLAM images  
- a **system health checklist** for the UNO Q  

Just tell me what direction you want to go next.