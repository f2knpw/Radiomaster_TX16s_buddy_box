# Radiomaster_TX16s_buddy_box
Radiomaster TX16s buddy box (master/trainer and BLE Joystick and more)

I wanted a wireless buddy box for my Radiomaster TX16s to wireless link two TX16s in Master/Trainer mode.


So I decided to build mine with the following specifications:
- cheap (two fully working modules for less than 20$)
- auto configuration as master or slave or vice versa
- auto binding between two modules
- fully hidden into the "external module" compartment (no dangling wire outside the radio))
- no modification into the TX16s
- no firmware modification of your TX16s (stock EdgeTx firmware)
- high speed wireless communication with about 20m range
- automatic On/Off of the buddy boxes via EdgeTx
- extensible to further options (teasing: more to come !)

  All this is open hardware and open source.
  The project is fully described on my Hackaday project page : https://hackaday.io/project/204529-tx16s-buddy-box-wireless-mastertrainer

  PCB is available on PCBWay shared project : https://www.pcbway.com/project/shareproject/Radiomaster_TX16s_buddy_box_master_trainer_and_more_9a7ddc8f.html

  It works with ESP32-C3 super mini boards. Source code is here : https://github.com/f2knpw/Radiomaster_TX16s_buddy_box/blob/master/ESP32_C3_TX16s_buddyBox.ino


a few videos showing how it works :

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Sv487SR5yGs 
" target="_blank"><img src="http://img.youtube.com/vi/Sv487SR5yGs/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

with an old TH9x PPM radio 
<a href="http://www.youtube.com/watch?feature=player_embedded&v=g21BqA_hWCk
" target="_blank"><img src="http://img.youtube.com/vi/g21BqA_hWCk/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

with 2 radiomaster TX16s  
<a href="http://www.youtube.com/watch?feature=player_embedded&v=TjBhkZSGRDY
" target="_blank"><img src="http://img.youtube.com/vi/TjBhkZSGRDY/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

and with a plane !  
<a href="http://www.youtube.com/watch?feature=player_embedded&v=onzTWL2fCzU
" target="_blank"><img src="http://img.youtube.com/vi/onzTWL2fCzU/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

and same board same code as a BLE Joystick !  
<a href="http://www.youtube.com/watch?feature=player_embedded&v=_bdqKs-VCjA
" target="_blank"><img src="http://img.youtube.com/vi/_bdqKs-VCjA/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>


And now a slight modification of the firmware (trainer side) to allow plugging a Thrusmaster T16000 joystick in an ESP32-S3 board and connect it wirelessly to my TX16s with the buddy box (original firmware onthe master side) 
Firmware source code here : https://github.com/f2knpw/Radiomaster_TX16s_buddy_box/blob/master/ESP32_S3_hid_host_Buddy_box_joystick.ino
<a href="http://www.youtube.com/watch?feature=player_embedded&v=oRUQ4mDlwY4
" target="_blank"><img src="http://img.youtube.com/vi/oRUQ4mDlwY4/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

