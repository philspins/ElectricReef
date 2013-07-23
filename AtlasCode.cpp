//pin outs:
//VCC=5V
//SI=Arduino Mega pin 2
//SO=Arduino Mega pin 3
//E=GND
//RX=Arduino Mega pin 14(TX3)
//TX=Arduino Mega pin 15(RX3)
//GND=GND



//defines:
int channel_tx(int channel,char* data);	//This is a function will open the correct channel and TX the data that is to be sent to it.
byte s_I=2;				//switch I is connected to pin 2.
byte s_O=3;   				//switch O is connected to pin 3.
byte  received_from_computer=0;		//we need to know how many characters have been received.
byte received_from_device=0;   		//we need to know how many characters have been received.
boolean data_parse = false;  		//has the data been parsed? 
char computerdata[20];			//we make a 20 byte character array to hold incoming data from a pc/Mac/other.
char device_data[40];			//we make a 40 byte character array to hold incoming data from the connected device.
int channel=99;				//we have 4 possible channels on the Mux/Demux(0-3) We set the channel var to 99
					//on boot up that way it won’t open a channel inadvertently

short ec=0;				//EC channel                           
short d_o=1;				//DO channel
short orp=2;				//ORP channel
short ph=3;				//PH channel

char *buffer_1;				//this buffer holds the channel open data.
char *buffer_2;  			//this buffer holds the data to be sent to the open channel.

boolean sensor_stringcomplete=false;
short ack_channel_read=0;  


//setup code:
void setup(){                                                                
     	Serial.begin(38400);		//set baud rate for the hardware serial port_0 to 38400
     	Serial3.begin(38400);		//set baud rate for the software serial port_3 to 38400
	pinMode(s_I, OUTPUT);
	pinMode(s_O, OUTPUT);
}



//functions:

//this interrupt will trigger when the data coming from
//the serial monitor (pc/Mac/other) is received.
 
void serialEvent() {                                                         
	received_from_computer=Serial.readBytesUntil(13,computerdata,20);	//we read the data sent from the serial monitor
										//(pc/Mac/other) until we see a <CR>. We also count
										//how many characters have been received. 
	computerdata[received_from_computer]=0;					//we add a 0 to the spot in the array just after the last
										//character we received. This will stop us from transmitting
										//incorrect data that may have been left in the buffer.

	data_parse = true;							//This flag will indicate that data has been received and 
										//it is now time to do something with it.    
}  


void serialEvent3() {                                                         
	received_from_device=Serial3.readBytesUntil(13,device_data,40);		//we read the data sent from the mux/demux serial port
										//connector until we see a <CR>. We also count
										//how many characters have been received. 
	device_data[received_from_device]=0;					//we add a 0 to the spot in the array just after the last
										//character we received. This will stop us from transmitting
										//incorrect data that may have been left in the buffer.
	Serial.println(device_data);						//we output the received data to serial monitor
}  


int channel_tx(int channel,char* data){  //this function will open a channel and transmit the data we have sent it. 
     
     switch (channel){                   //which channel do we open? 0-3.
           
        case 0:                          //we are to open channel Y0.                         
           digitalWrite(s_I,LOW);        //Y0 is opened with SI=0.
           digitalWrite(s_O,LOW);        //Y0 is opened with SO=0.
           Serial3.print('\r');          //We send a <CR> first to clear out any junk that was left in the RX buffer of an Atlas Scientific device. Data noise can happen when switching between channels.
           Serial3.print(data);          //Send the data.
           Serial3.print('\r');          //All data going to an Atlas Scientific device if transmitted with a <CR>.
        break;                           //Exit this case.
     
   
 
        case 1:                          //We are to open channel Y1. 
           digitalWrite(s_I,HIGH);       //Y1 is opened with SI=1. 
           digitalWrite(s_O,LOW);        //Y1 is opened with SO=0. 
           Serial3.print('\r');          //We send a <CR> first to clear out any junk that was left in the RX buffer of an Atlas Scientific device. Data noise can happen when switching between channels.
           Serial3.print(data);          //Send the data.
           Serial3.print('\r');          //All data going to an Atlas Scientific device if transmitted with a <CR>.
        break;                           //Exit this case.
        
        
        
         case 2:                        //We are to open channel Y2.
           digitalWrite(s_I,LOW);       //Y2 is opened with SI=0.
           digitalWrite(s_O,HIGH);      //Y2 is opened with SO=1.
           Serial3.print('\r');         //We send a <CR> first to clear out any junk that was left in the RX buffer of an Atlas Scientific device. Data noise can happen when switching between channels.
           Serial3.print(data);         //Send the data. 
           Serial3.print('\r');         //All data going to an Atlas Scientific device if terminated with a <CR>. 
        break;                          //Exit this case.
 
 
        case 3:                        //We are to open channel Y3.
           digitalWrite(s_I,HIGH);     //Y3 is opened with SI=1.
           digitalWrite(s_O,HIGH);     //Y3 is opened with SO=1. 
           Serial3.print('\r');        //We send a <CR> first to clear out any junk that was left in the RX buffer of an Atlas Scientific device. Data noise can happen when switching between channels.
           Serial3.print(data);        //Send the data. 
           Serial3.print('\r');        //All data going to an Atlas Scientific device if terminated with a <CR>.
        break;                         //Exit this case.
         
 
     }
  }


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






//code for main loop:

void loop(){ 
  int channel_tx(int channel,char* data){
  if(data_parse==true){
     buffer_1=strtok(computerdata,",");
     buffer_2=strtok(0,",");
     channel=atoi(buffer_1);
     data_parse=false;  
     channel_tx(channel,buffer_2);
     }
  }



//alternative:
void loop(){ 
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
}
