# 🌐 WSL2 Mirrored Networking Mode — Full Configuration

Mirrored mode makes WSL2 share the host’s network identity, meaning:
•	WSL2 uses the same LAN IP as Windows
•	Outbound multicast works
•	Unicast DDS works perfectly
•	No virtual switch required
•	No Hyper V configuration
•	No special permissions

It’s the easiest way to improve DDS reliability on WSL2.
________________________________________
## 🟦 1. Create or Edit .wslconfig
Open:
```
C:\Users\<your-username>\.wslconfig
```
Add exactly this:
```
[wsl2]
networkingMode=mirrored
```
That’s all you need.
If you already have memory/swap settings, just append the networking line:
```
[wsl2]
memory=25769803776
swap=8589934592
networkingMode=mirrored
```
________________________________________
## 🟩 2. Restart WSL Completely
In PowerShell or CMD:
```
wsl --shutdown
```
Then launch WSL again:
```
wsl
```
This forces WSL2 to rebuild its network stack using mirrored mode.
________________________________________
## 🟧 3. Verify Mirrored Mode Is Active
Inside WSL2:
```
ip addr show eth0
```
You should now see an IP that matches your Windows host, for example:
```
inet 192.168.1.ZZ/24
```
If you see a 172.x.x.x address, mirrored mode is NOT active.
________________________________________
## 🟦 4. Test Connectivity With the UNO Q
From WSL2:
```
ping 192.168.1.XXX
```
From UNO Q:
```
ping 192.168.1.YY
```
Both should work.
If they do, DDS unicast will work flawlessly.
________________________________________
## 🟩 5. DDS Behavior in Mirrored Mode
Mirrored mode improves DDS in several ways:

✔ Outbound multicast works
FastDDS can send announcements.

✔ Unicast discovery works perfectly
Your XML files remain valid.

✔ No more WSL2 NAT issues
UDP flows directly to the LAN.

✔ Docker host networking becomes simpler
Containers inherit the mirrored IP.

✔ No need to update WSL2 IP
It stays the same as your Windows host.

________________________________________
## 🟧 6. What You Still Need for DDS
Even in mirrored mode, you should continue using unicast FastDDS XML:
- WSL2 → UNO Q
- UNO Q → WSL2
- 
Because inbound multicast is still unreliable on some Wi Fi networks.

Your existing XML files remain correct.
________________________________________
## 🟦 7. Quick Diagnostic Script (optional)
Inside WSL2:
```
echo "WSL2 IP:" && ip -4 addr show eth0 | grep inet
echo "Windows IP:" && ipconfig | findstr /R /C:"IPv4"
```
