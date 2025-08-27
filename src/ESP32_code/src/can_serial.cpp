#include "can_serial.h"

CanSerial::CanSerial(Stream &serial) {
  _serial = &serial;
}

void CanSerial::begin(unsigned long baudrate) {
  if (&Serial == _serial) {
    Serial.begin(baudrate);
  }
}

bool CanSerial::sendPacket(const DataPacket &packet) {
    size_t numByteSend = 0;
    uint8_t header = 0x2a;
    uint8_t tail = 0x23;

    numByteSend += _serial->write(&header, 1);
    // Gửi ID (4 bytes)
    numByteSend += _serial->write((uint8_t*)&packet.id, sizeof(packet.id));
    // Gửi 8 bytes dữ liệu
    numByteSend += _serial->write(packet.data, 8);

    numByteSend += _serial->write(&tail, 1);

    return numByteSend == 14;
}

bool CanSerial::readPacket(DataPacket &packet) {

    if (_serial->available()){
        while (_serial->available()) {
            uint8_t data = _serial->read();
            // Serial2.println(data);
            // Serial2.println("-----------------");

            if (_rxIndex == 0 && data != 0x2a){
                _rxIndex = 0;
                break;
            }

            else if (_rxIndex == 13 && data != 0x23){
                _rxIndex = 0;
                break;
            }

            if (_rxIndex == 13 && data == 0x23){

                memcpy(&packet.id, _rxBuffer, 4);
                memcpy(packet.data, _rxBuffer + 4, 8);
                _rxIndex = 0;
                
                // for (int i = 0; i < 12; i++) {
                //     Serial2.print(_rxBuffer[i]);
                //     Serial2.print(" ");
                // }
                // Serial2.println("-----------------");

                return true;

            }else{
                if (_rxIndex > 0){
                    _rxBuffer[_rxIndex-1] = data;
                }
                _rxIndex++;
            }
        }
    }

    return false;
}

// bool CanSerial::readPacket(DataPacket &packet) {
//     if (_serial->available()) {
//         uint8_t data = _serial->read();

//         Serial.println(data);

//         if (_rxIndex == 0) {
//             if (data == 0x2a) {
//                 _rxBuffer[_rxIndex++] = data;
//             }
//         } else {
//             _rxBuffer[_rxIndex++] = data;

//             if (_rxIndex == 14) {
//                 _rxIndex = 0;

//                 if (_rxBuffer[13] == 0x23) {
//                     memcpy(&packet.id, _rxBuffer + 1, 4);
//                     memcpy(packet.data, _rxBuffer + 5, 8);
//                     return true;
//                 }
//             }
//         }
//     }

//     return false;
// }