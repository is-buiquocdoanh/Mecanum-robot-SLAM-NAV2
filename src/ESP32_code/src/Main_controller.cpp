#include "Main_controller.h"

Main_controller::Main_controller(CAN_manager* Main)
{
    CAN_main = Main;
}

Main_controller::~Main_controller()
{
    delete send_CAN;
    delete received_CAN;
}

bool Main_controller::CANTransmitHandle() {
    CAN_main->SetIDTransmit(send_CAN->id);
    CAN_main->SetByteTransmit(send_CAN->byte0, 0x00);
    CAN_main->SetByteTransmit(send_CAN->byte1, 0x01);
    CAN_main->SetByteTransmit(send_CAN->byte2, 0x02);
    CAN_main->SetByteTransmit(send_CAN->byte3, 0x03);
    CAN_main->SetByteTransmit(send_CAN->byte4, 0x04);
    CAN_main->SetByteTransmit(send_CAN->byte5, 0x05);
    CAN_main->SetByteTransmit(send_CAN->byte6, 0x06);
    CAN_main->SetByteTransmit(send_CAN->byte7, 0x07);
    if (CAN_main->CAN_Send()) {
        return 1;
    }else{
        return 0;
    }
}

void Main_controller::CANReceiveHandle() {
    uint32_t addRevFromCanBus;
    // -------
    addRevFromCanBus = CAN_main->CAN_ReceiveFrom();
    if(addRevFromCanBus != 0x55) // 0x55 | 85
    {
        saveTime_checkCAN = millis();
        received_CAN->idSend = addRevFromCanBus;
        received_CAN->byte0 = CAN_main->GetByteReceived(0x00);
        received_CAN->byte1 = CAN_main->GetByteReceived(0x01);
        received_CAN->byte2 = CAN_main->GetByteReceived(0x02);
        received_CAN->byte3 = CAN_main->GetByteReceived(0x03);
        received_CAN->byte4 = CAN_main->GetByteReceived(0x04);
        received_CAN->byte5 = CAN_main->GetByteReceived(0x05);
        received_CAN->byte6 = CAN_main->GetByteReceived(0x06);
        received_CAN->byte7 = CAN_main->GetByteReceived(0x07);
        is_receivedCAN = true;
        sts_LED_CAN_RECEIVED = true;
    }
}

void Main_controller::led_loop(){
    if (sts_LED_ROS_SEND == true){
        saveTime_LED_ROS_SEND = millis();
        sts_LED_ROS_SEND = false;
    }
    if ((millis() - saveTime_LED_ROS_SEND) < 100 ){
        digitalWrite(PIN_LED_ROS_SEND, 0); // - Lighting
    }else{
        digitalWrite(PIN_LED_ROS_SEND, 1);
    }
    // -
    if (sts_LED_ROS_RECEIVED == true){
        saveTime_LED_ROS_RECEIVED = millis();
        sts_LED_ROS_RECEIVED = false;
    }
    if ((millis() - saveTime_LED_ROS_RECEIVED) < 100 ){
        digitalWrite(PIN_LED_ROS_RECEIVED, 0); // - Lighting
    }else{
        digitalWrite(PIN_LED_ROS_RECEIVED, 1);
    }
    // - CAN
    if (sts_LED_CAN_SEND == true){
        saveTime_LED_CAN_SEND = millis();
        sts_LED_CAN_SEND = false;
    }
    if ((millis() - saveTime_LED_CAN_SEND) < 100 ){
        digitalWrite(PIN_LED_CAN_SEND, 0); // - Lighting
    }else{
        digitalWrite(PIN_LED_CAN_SEND, 1);
    }
    // - 
    if (sts_LED_CAN_RECEIVED == true){
        saveTime_LED_CAN_RECEIVED = millis();
        sts_LED_CAN_RECEIVED = false;
    }
    if ((millis() - saveTime_LED_CAN_RECEIVED) < 100 ){
        digitalWrite(PIN_LED_CAN_RECEIVED, 0); // - Lighting
    }else{
        digitalWrite(PIN_LED_CAN_RECEIVED, 1);
    }
}
void Main_controller::led_blink(){
    if ( (millis() - saveTime_light) > 400){
        saveTime_light = millis();
        if (sts_light == 0){
            sts_light = 1;
        }else{
            sts_light = 0;
        }
    }

    if (sts_light == true){
        digitalWrite(PIN_LED_ROS_RECEIVED, 1);    
    }else{
        digitalWrite(PIN_LED_ROS_RECEIVED, 0);    
    }
    
}

void Main_controller::led_control(bool sts1, bool sts2, bool sts3, bool sts4){
	digitalWrite(PIN_LED_ROS_SEND, sts1);
    digitalWrite(PIN_LED_CAN_RECEIVED, sts2);
	digitalWrite(PIN_LED_CAN_SEND, sts3);
    // digitalWrite(PIN_LED_ROS_RECEIVED, sts4);    
}

void Main_controller::led_start(){
	digitalWrite(PIN_LED_ROS_SEND, 1);
    digitalWrite(PIN_LED_CAN_RECEIVED, 1);
	digitalWrite(PIN_LED_CAN_SEND, 1);
    digitalWrite(PIN_LED_ROS_RECEIVED, 1);
	delay(10);
    digitalWrite(PIN_LED_ROS_SEND, 0);
	delay(60);
    digitalWrite(PIN_LED_ROS_SEND, 1);
    digitalWrite(PIN_LED_CAN_RECEIVED, 0);
	delay(60);
    digitalWrite(PIN_LED_CAN_RECEIVED, 1);
	digitalWrite(PIN_LED_CAN_SEND, 0);
    delay(60);
	digitalWrite(PIN_LED_CAN_SEND, 1);
    digitalWrite(PIN_LED_ROS_RECEIVED, 0);
    delay(60);
    digitalWrite(PIN_LED_ROS_RECEIVED, 1);
    delay(100);
	digitalWrite(PIN_LED_ROS_SEND, 0);
    digitalWrite(PIN_LED_CAN_RECEIVED, 0);
	digitalWrite(PIN_LED_CAN_SEND, 0);
    digitalWrite(PIN_LED_ROS_RECEIVED, 0);
}

void Main_controller::init_main(){
    WiFi.mode(WIFI_OFF);
    btStop();
    pinMode(PIN_LED_CAN_RECEIVED, OUTPUT);
    pinMode(PIN_LED_CAN_SEND,     OUTPUT);
    pinMode(PIN_LED_ROS_RECEIVED, OUTPUT);
    pinMode(PIN_LED_ROS_SEND,     OUTPUT);
    pinMode(PIN_INPUT1, INPUT);
    pinMode(PIN_INPUT2, INPUT);
    //-------------------------------------------- OUT ------------------ 
}

void Main_controller::Run_Send_CAN(){
    // - send CAN
    if (is_sendCAN == true){
        if ( (millis() - saveTime_sendCAN) > 10){
            saveTime_sendCAN = millis();
            sts_LED_CAN_SEND = CANTransmitHandle();
            is_sendCAN = false;
        }
    }
}

int Main_controller::bytes_to_int(uint8_t byte0, uint8_t byte1){ // for 2 bytes
    return byte0 + byte1*256;
}

uint8_t Main_controller::int_to_byte0(int value){ // for 2 bytes
    
    return value - (value/256)*256;
}

uint8_t Main_controller::int_to_byte1(int value){ // for 2 bytes
    
    return value/256;
}

bool Main_controller::getBit_fromInt(int val, int pos){
    bool bit;
    int val_now = val;
    for (int i = 0; i < 8; i++){
        bit = val_now%2;
        val_now /= 2;

        if (i == pos){
            return bit;
        }

        if (val_now < 1)
            return 0;
    }
    return 0;
}
