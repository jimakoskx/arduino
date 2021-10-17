# arduino
English manual of the most upgrated and stable version of my program.

For greek manual and greek yt videos see
https://jimakoskx.blogspot.com/2021/01/arduino-beehive-5-scales-dht-thingspeak.html

Arduino Pro Mini Program -8MHz running on about 4-4,1 volts with regulator-
for
5 (max) Beehive Scales sending data to Thingspeak (and/or Sms) via SIM800L module
and having also LCD 2x16 (minimum rows columns)
for adjustements on the below lcd menu .

The program can run 'satisfyinglly enough' 
even with just 3 AA batteries (i use rechargable varta)
but my box can have 4 parallel sets of 3 AA...
And i have made it possible to AUTO POWER-OFF if we detect that we 
are in some LOW VOLTAGE and we dont want to damage our rechargables.
(especially if someone wants lithion batteries)
And ofcource we throw/disassemble the static led of arduino pro mini. 




 

Important Note

a)
10 pins for 5 possible scales (2 each HX711 amplifier)
2 pins for sim (rx tx)
2 pins for sim latch relay (set reset)
1 pin for power off via reset pin of main latch relay
2 pins for buttons
2 pins for lcd (A4 A5)
1 pin for reading battery
= 20 pins !!!(23456789,10,11,12,13,A0,A1,A2,A3,A4,A5,A6,A7)

Meaning that DHT can NOT be used  if we have 5 scales !
(So except if some trick can be programmed !?!?) 
we can have DHT and 4 scales....
5th scale .... no DHT pin space !!!

b)
I have NOT yet messed up with possible too much  COVARIANCE/fluctuation
of the Weight Measurments Relative to Temperature Changes.
In case that this is a big problem then i thing that this can be 'fixed'
from Thingspeak itself meaning
....analyse correlations and create tables(views)
that trying to show the real weights.(and not the affected from temperature)
After all,,, the DIFFERENCES on weights matter (how much new honey)
and temperatures -most propably-does not flunctuate too much 
for the same time on 2 continues days..


Onother Note
The program is NOT USING RTC even if in the schematics
is 'ready to go' for someone that wants.
This is just for being very simple....even if we may loose 1,2,3,seconds
(depending our adjustement-see menu) 
on every step....(but who cares for those...not me at least)
Most propably someone wants to take the weights every 1 or 2 hours ...
not carrying if the 1st reading of the 1st day lets say come at 9:00
and the 2nd day comes at 9:01 (+1 minute)...
Anyway ....this can be
almost perferctlly be adjusted without RTC ...(see menu)

The simplicity also makes us running at about 4,1 volts
(using an adjustable step up/down converter
personally i am using pololu 2868 or 2869)

in order to have latch relays without some transistor .
(It is almost unlikelly that we can find any relay for ~3 volts with
coil resistance so that the current does not exceed 40 ma for arduino pins)
So its very usable that we can find relays 5 volts
to be used without some Resistor in Base pin.
(directly from arduino pin to set/reset pin of the relay)
A 5 volt relay with coil 125 Ω is in the limit of 40 ma
but accepted if we will run at ~4 volts.
(But you can easy find 5 volt relay with coil bigger than 125)


And those 4.1 volts are ALSO perfect for SIM800L module
and is enough to trigger the 5 volt relays !!
 
 
 
 
 
 
 
 
 
 
 
 
 
 (Parenthesis,,,
Beside the 2 momentary buttons for on/off the main latch relay-powering arduino-
and beside the 1 latch button for LCD powering when we are in adjusting mode
(adjusting mode vs coffe mode in our home away from farm/field waiting the measurements)...................
We have 2 buttons attached to arduino pins for lcd adjustements.
But we have 4 functions (2 functions for each button).
-press and release quicker than 666 milliseconds (1st function)
-press and release after 666 milliseconds (2nd function)
So 
Button A quick release = Menu Roll
Button A slow release = Ok (store current settings ..or do something)
Button B quick release = Next digit (from '0' to '1'etc ) in current position of Cursor.
Button B slow release = Next position of Cursor in the number-word that we are editing.
Parenthesis End )





--------------------------
The function of the arduino applicatiion is simple.
In order to adjust some things in lcd mode
we powering up the arduino and we ensure the the latch button of lcd
is pressed so the lcd have power.
Like this the arduino starts and we are in the menu mode.

After that we have make all the adjustements and
we want to leave the arduino in the farm/field with beehives 
we close the arduino and 
we opening again but pressing also the Menu/Ok button 
meaning that if arduino detects that some button (adjusting button)
is pressed by the time that is powering on
then we will not be on the 'LCD LOOP' (to make adjustements)
but on the 'Coffe loop' !!!! meaning we can go home
waiting the measurements to Thingspeak .
So for Coffe mode we just
a)press the Ok button (no release)
b)push the Main Power Button (it is momentary and not latch button)
c)wait 1-2 seconds and release the Ok button
d)Wait (~1 minute max?) to check on Thingspeak
that arduino INDEED send the fresh data...
e)And if we see that thingspeak has just now been updated
we just ensuring lcd is powered OFF (to not loose battery power)
and we are going for home/coffee!!!!....
waiting the measurements from load scales
every 1 (or 2 or 3 ,etc ) ,,, hours (what ever we have adjust)
...to be updated on thingspeak !!!!

---------------------------------------------

I am using 2 LATCH relays (2 coils latch relays)

The first latch relay is the one that we are triggering(set pin) to power up the hole application.
And is latch relay (not normal) to be able to trigger the reset pin
somewhere from the program to 'auto' power off the application.
And the second latch relay is the one that we are triggering
from the program to power the sensors and sim800l module.
And is latch  relay (not normal) just to minimize battery power consumption.
(summarized milli-amperes of holding a normal relay triggered
for 30-40 seconds that needs a step to send the data on thingspeak
for lets say 60 days and 12 measurements per day
is.....something....even if is not a big amount....but it is something
especially for someone that wants to use simple AA rechargables batteries
that can be used with safety and in many other applications
instead of lithion batteries................. )

Here are some examples of 2 coils latch relays that can
can be used and handle 2A (as per max current of SIM800L)

Omron G6AK-234P-ST(40)-US  (coil 139 mean ~36 ma)
Omron G6AK-274P-ST(40)-US
Panasonic TQ2-L2-5V                 (coil 178 mean ~28 ma)
Axicom V23079-B1202-B301     (coil 178 mean ~28 ma)
Zettler  Z850P2-5                         (coil 125 mean ~40 ma)

(
Zettler is good (and cheap) to be used as MAIN POWER relay 
because i think can be triggered (directly from battery)
even below 3.5 volts (holding button pushed a little more time)
...even if the purpose of the application is to be able
to power on with 3AA charged good =4-4.3 volts is enough for triggering.
I mean ....if someone wants lithium batteries that have max 3.7 volts
...maybe wants to use a 3 volts relay ....but must change a little the scheme
)


The second relay (sim-sensors) have also 1 more function.
One channel (lets say left side)of its pins is used ok..for sim-sensors.
But the other channel (lets say right side)
is used for connecting the battery
to some analogue pin of arduino and have it as reference
to be able to know the current voltage.
So only when we triggering the relay
(powering up sensors and sim for about 40 seconds)
we are loosing some current
(even if ok...i have Mega-Ohm resistor there)
avoiding have continiuslly some current to flow out !!!

--------------------------------------
How we measure/ translate real voltage?

The time that
sim is powered on and the analogue pin is also connected to battery
(via sensor-sim relay)
we ask sim (with AT+CBC command) on how many volts is running.
Sim is attached to the converted by step up/down voltage.
(left channel-side of relay)
But analogue pin is attached to battery voltage. 
(right channel-side of relay)

And the answer we get is the ACTUAL
(and current because it is flunctuating a little)
output of the step up/down concverter .

(I am adjusting my adjustable step up/down converter
somewhere around 4.1 volts)

Now if the batteries have more voltage than 4.1
(lets say in the beginning 3 AA will have 4.3 volts)
(meaning that the regulator works as step DOWN converter)
then we will NOT find the real voltage but who cares?
(because we want to know the voltage only if its very low
to save our rechargable batteries)
We will thing that the real voltage is just the stepped down voltage
meaning the voltage that sim is working.

But if the batteries have lets say 3.2 volts
(meaning that the regulator works as step UP converter)
then with a simple analogy we are translating the analogue value
and we find the real voltage  !

You can see below that in the beginning we are not seeing the true voltage.
(Line is straight till real voltage becomes lower than converted)








------------------------------------------------------------ 

This is also the reason that just for testing/debugging 
reasons i am binding 2 fields on Thingspeak channel
(you can see in above picture that we have
3.784 real volts and sim is running at 3.9 volts)
The client does not need to know the sim voltage
(the converted voltage is only for debugging/testing)
The client may only need to know the real voltage
(to understand that in the next visit on the farm field with bees
he must have with him a new set of batteries to change if its very low)
Irrelative if arduino will auto power off depending the adjustements.

At the time of writing this,,,
when we are creating a new channel on Thingspeak
(after creating an account )
the channel has 8 fields
(that in the beginning we must enable them all)
(and we must update -from lcd mode-arduino eeprom with the WRITE API KEY)
 
5 fields are for the 5 load scales
1 field is for the combined temperature and humidity
    example 1751 means 17 Celsiu 51 Humidity (who cares for humidity..)
1 field is for the real voltage 
1 field is remaining for programmer to change the code and use it as he wants.
     I personally use the last field to have also the signal strenght  for debug
   (this added in the last upgrate of program...when i also add 1500uf capacitor)
     example field8=263999 means last signal = 26 and 3.9 last sim working
-------------------------------------









Menu#1
- Enter program password and 'press' Ok 
(
that is the same with the PIN of the phone card irrelative if pin usage is enabled
This will not mess with the phone card meaning will not lock the card in case of error.
It is just a precausion....that if we dont afraid of someone mess with the box in the farm
....we can just set the password 1111 (via phone card pin menu)
Just that like this ...in case of stolen we gona loose the money in card...
But who cares...for 10 pre paid euros in this case?..that we will loose the hole beehive scale!
Anyway...just an effort....
)





Menu#2
-Enter in second line  the ... every how many Minutes and Second
 we want the arduino to get-off from the Deep Sleep (many times of 8 seconds deep sleep)
and opens the latch relay that gives power to Sim800L and Sensors
(HX711 from load cells and Dht)
taking the data and sending them to ThingSpeak and or Sms.

In first line (Just a non critical future of the program )
we entering how many hours we want the process to delay.
This is because maybe lets say user wants every 2 hours but wants also not to start immidiatelly.




Menu#3
Showing the current data of load cell sensors and Dht (temperature-humidity)
Here also we can Tare some load cell .
Press Ok to tare the current scale.(if the scale is empty ofcource)
Press Next Cursor to iterate through 5 possible scales
Press Next Digit to just switch on/off if we want to see ALSO
the output of HX711 amplifier ...beside the translation in grams /units
Below we see that scale #0 counts 747 grams (or units dependig your callibration)
and its tare (the amount adjusting the result) is zero.
Also we have temperature 18 Celsiu and 58% humidity. 


....continuing menu/manual tomorrow !!!...i am boring...!(17 Sep 2021!)





















The schematic below....

a)
and you need ALSO
a >=1500 uF (electrolytic capacitor-not in below schematics..)
after batteries and before step up/down(most propably)
in order to have GOOD SIGNAL .

b)
3+3 pins on each side of pro mini
(Tx,Rx,Rst + Vin,Grnd,Rst)
ARE NOT connected to breadboard
neither we are using them at all
10 pins for 5 possible scales
2 pins for sim
2 pins for sim relay
1 pin for power off from main relay
2 pins for buttons
2 pins for lcd 
1 pin for reading battery
= 20 pins !!!
Meaning that DHT can be used only if we have less than 5 scales!
There is no DHT in scheme....use some hx711 pin or make your tricks !!!



c)In below image ,,, about in the middle of up left quarter
you can see a black 1x3 holes 'breaking' the orange line.
(By the way orange means the converted output from regulator)
This 1x3 is just a tiny switch that we use in debug/test
meaniing that if we are building via usb we dont want
usb to touch also the batteries...and thats why tiny switch is there.

d)In the scheme i have only 2 molex for batteries (2x3 holes)
but right of them we have space also for 2 more meaning
i can support 4 parallel sets of ...(3 AA batteries personally)

e)You can see that the red line (battery +) coming out from main relay(zettler)
goes not only to step up/down converter (as VIN)
but also to the second channel of sensor relay so that when triggered 
red line is touching analogue pin A6 !

f)Except relays.. all the others ... i have them with header pins
in my...prototype....
So in image below the rounded rectangles of
-arduino
-voltage regulator
-sim800l
-hx811
-lcd
-possible rtc
-and keypad
are not directly on breadboard but via pin headers.
Especially arduino has doubled pin headers by the thinking that
maybe someday i may want to do some ...experiments...and have not only 
the females pins (from the doubled pin headers)
but also arduino has long-normal male-male pins
that those males go to females that females is stable on breadboard.








Before the code in oder to be able to adjust the DHT to 11 or 12 or 21 or 22 
(even if it is just a simple future of the program and i have test and use only dht 11)
we need to change the private access to public
in the uint8 _type field of dht library
for succesfully compiling

(Or you can just erase the private:  and save the file..i think its ok...)


The Code below .....  has many comments for someone to understand !

Supports SMS to be send every 4 measurements but this
was the purpose when i begun the project Sep 2020.
After we saw that sms is very expensive
i find (2021) the thingspeak solution that is VERY CHEAP
(may be only 10 euros per year...here in Greece-vodafone)
and added also some SMS support ...but i dont use sms ..anyway.

--------------------------------------------------





 
