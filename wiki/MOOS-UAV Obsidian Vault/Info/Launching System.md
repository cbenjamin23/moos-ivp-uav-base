

## SSH to odroid

On my computer, the ifconfig yields

```
enx00e04c680055: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255
        inet6 fe80::d0dd:1e97:db04:6767  prefixlen 64  scopeid 0x20<link>
        ether 00:e0:4c:68:00:55  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 32  bytes 4804 (4.8 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

```

with `enx00e04c680055` for the odroid.

I have set the IPv4 adress on this ethernet interface to 

```
Address: 10.0.60.120, 
Netmask (same as on the odroid): 255.255.0.0
```

This enables me to ssh in to the odroid with the command 

	ssh odroid@10.0.60.110

Add an ssh key for simplicity of loging in (more instructions [here](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Help.SSHKeys))

**Login credentials**:
```
 username: odroid
 password: moosuav2024
```



## Internet sharing

`set_cusom_ip.sh` is located in `/usr/local/bin` and sets the ip of the ethernet gateway to the odroid to the gateway IP.

```
#!/bin/bash
# Set custom IP after enabling Internet Sharing
sudo ip addr flush dev enx00e04c680055
sudo ip addr add 10.0.60.1/16 dev enx00e04c680055
sudo ip link set enx00e04c680055 up
```

This can also be done in the GUI network setting. Remember that the mask `/16` means `255.255.0.0`

After this, enable forwarding to internet by calling the script

	sudo /usr/local/bin/setup_nat_ip_forwarding.sh

containing  

```bash
#!/bin/bash
# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# Enable NAT for Wi-Fi (wlp1s0)
sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE
sudo iptables -A FORWARD -i enx00e04c680055 -o wlp1s0 -j ACCEPT
sudo iptables -A FORWARD -i wlp1s0 -o enx00e04c680055 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Enable NAT for Wired Ethernet (enx4865ee197242)
sudo iptables -t nat -A POSTROUTING -o enx4865ee197242 -j MASQUERADE
sudo iptables -A FORWARD -i enx00e04c680055 -o enx4865ee197242 -j ACCEPT
sudo iptables -A FORWARD -i enx4865ee197242 -o enx00e04c680055 -m state --state RELATED,ESTABLISHED -j ACCEPT
```

Adapt the wifi interfaces to your specific interface.


# Launching System


## Launching vehicle

Use command:

```bash
./launch_vehicle.sh --shore=10.0.60.1 --shore_pshare=9200 --ip=10.0.60.110 --pshare=9201 --vname=skywalker -v
```

and (for more debugging add `MAVSDK_MESSAGE_HANDLER_DEBUGGING=1 `)

```bash
MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 pArduBridge targ_skywalker.moos
```

## Launching Groundstation

```bash
./launch_shoreside.sh --ip=10.0.60.1 --pshare=9200 --vname=all:skywalker -v
```