//pin outs:
//VCC=5V
//SI=Arduino Mega pin 2
//SO=Arduino Mega pin 3
//E=GND
//RX=Arduino Mega pin 14(TX3)
//TX=Arduino Mega pin 15(RX3)
//GND=GND



//defines:
String inputstring = "";    	//a string to hold incoming data from the PC
String sensorstring = "";		//a string to hold the data from the Atlas Scientific product                           
boolean input_stringcomplete = false;	//have we received all the data from the PC
boolean sensor_stringcomplete = false;	//have we received all the data from the Atlas Scientific
int channel_tx(int channel,char* data);	//This is a function will open the correct channel and TX the data that is to be sent to it.


//setup code:
void setup(){                                                                
     Serial.begin(38400);	//set baud rate for the hardware serial port_0 to 38400
     Serial3.begin(38400);	//set baud rate for the software serial port_3 to 38400
     inputstring.reserve(5);	//set aside some bytes for receiving data from the PC
     sensorstring.reserve(30);	//set aside some bytes for receiving data from the probe
}



//functions:

//opens channels of the serial mux/demux chip
void Open_channel(short channel){
    switch (channel) {
    
      case 0:                                 
        digitalWrite(4, LOW);                     
        digitalWrite(5, LOW);                  
      break;
      
      case 1:                                  
       digitalWrite(4, HIGH);                      
       digitalWrite(5, LOW);                   
      break;
      
      case 2:                                     
       digitalWrite(4, LOW);                    
       digitalWrite(5, HIGH);                 
      break;
      
      case 3:                                  
       digitalWrite(4, HIGH);                      
       digitalWrite(5, HIGH);                  
      break;
    }   


//opens data from open channel of the serial mux/demux chip
short read_channel(short channel){
	if (sensor_stringcomplete){                                                 
		delay(1100);                            //let 1.1 sec pass
		myserial.print("r");   
		myserial.write(13);
		while (myserial.available()) {                                               
         		char inchar = (char)myserial.read();                                  
         		sensorstring += inchar;                                               
         		if (inchar == '\r') {sensor_stringcomplete = true;}                   
         	}
 		if(channel==0){
          		Serial.print("EC: ");
          		ack_channel_read=1;
          	}
       
       		if(channel==1){
         		Serial.print("D.O: "); 
         		ack_channel_read=2;
         	}
       
       		if(channel==2){
          		Serial.print("ORP: "); 
          		ack_channel_read=3;
          	}
       
       		if(channel==3){
          		Serial.print("Ph: "); 
          		ack_channel_read=4;
          	}  
 		Serial.print(sensorstring);                                             
       		sensorstring = "";                                                      
       		sensor_stringcomplete = false;                                          
      	}
    
	return ack_channel_read;
}



void serialEvent() {                                                         
	char inchar = (char)Serial.read();			//if the hardware serial port_0 receives a char
	inputstring += inchar;					//get the char we just received & add it to the inputString
        if(inchar == '\r') {input_stringcomplete = true;}	//if the incoming character is a <CR>, set the flag
}  










//code for main loop:

while(ack_channel_read==0){
	Open_channel(ec);                         
	ack_channel_read=read_channel(ec);
}
      
while(ack_channel_read==1){
	Open_channel(d_o);                      
	read_channel(d_o);
}
   
while(ack_channel_read==2){
	Open_channel(orp);                      
	read_channel(orp);
}
          
while(ack_channel_read==3){
	Open_channel(ph);                      
	read_channel(ph);
}


if(ack_channel_read==4){ack_channel_read=0;}
