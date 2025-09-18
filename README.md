# J2534 on Linux

*Trying this code out on a Raspberry Pi. Heres what happened.*


Run the python command to attempt connecting and sending a message.  

`sudo python3 j2534_send.py --protocol can --baud 500000 --id 0x7DF --data 02 01 00 --wait 1000`
