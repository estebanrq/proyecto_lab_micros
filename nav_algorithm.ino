//
//
//
//
//

// motorA_F
// motorA_B
// motorB_F
// motorB_B
// ultras_sendF
// ultras_receiverF
// ultras_sendL
// ultras_receiverL
// ultras_sendR
// ultras_receiverR
// TH_data
// encodeA_write
// encodeA_read
// 
//
//

float get_distance(int ult_W, int ult_R);

//GET_distances()
//    leer utrasound1
//        send 
//        delay
//        recive
//            calculo de distancia1
//    leer utrasound2
//        send 
//        delay
//        recive
//            calculo de distancia2
//    leer utrasound3
//        send 
//        delay
//        recive
//            calculo de distancia3
//
//
//COMPARE L AND R DISTANCES()
//    if |distR - distL| > threshold:
//        if distR > distL:
//            doblar_izquierda
//                motorA_F = 5
//                motorA_B = 0
//                motorB_F = 2
//                motorB_B = 0
//        else if distR > distL:
//            doblar derecha
//                motorA_F = 2
//                motorA_B = 0
//                motorB_F = 5
//                motorB_B = 0
//    else    
//        directo
//
//READ_TH()
//    leer sensor TH

float ultras_sendF;
float ultras_recieveF;
float ultras_sendL;
float ultras_recieveL;
float ultras_sendR;
float ultras_recieveR;


float dF;
float dL;
float dR;



void setup(){

}


void loop(){
    dF = get_distance(ultras_sendF, ultras_recieveF);    
    dL = get_distance(ultras_sendL, ultras_recieveL);
    dR = get_distance(ultras_sendR, ultras_recieveR);    

}


float get_distance(int ult_W, int ult_R){
    digitalWrite(ult_W, HIGH);
    delayMicroseconds(2);
    digitalWrite(ultrasA, LOW);
    delayMicroseconds(10);
    float duration = pulseIn(ult_R, HIGH);
    delay(100);
    return duration * 0.034 / 2;
}