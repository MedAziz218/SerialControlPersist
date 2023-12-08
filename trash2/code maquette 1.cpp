void loop()
{
  if (lampa.ON == 1)
  {
    debugTimer = millis();
    debugLog("Bdina");
    for (int i = 0; i < 3; i++)
    {
      if (1)
      {
        // code el maquette
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.setLineColor(BLACK_LINE);
        if (i == 0 || i==2)
        {
          if (i==0){

          debugLog("start point");
          lampa.setPIDNum(PID_0);
          // Accelerate ------;
          lampa.setSpeed(50);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(75);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(100);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(120);
          lampa.forwardUntilDelayOrEncoder(0, 226, 226);

          // -----------------;

          }
          lampa.followLineUntilSensor(SENSOR_RIGHT);
          lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right
          lampa.followLineUntilDelayOrEncoder(125, 0, 0);         // cooldown
          debugLog("kamalt awal dora right");

          // debugPause(200);
          lampa.followLineUntilSensor(SENSOR_RIGHT_CENTER); // detecti moftara9
          // debugPause(1000);
          lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 90); // dour el moftara9 encL:3 encR:95
          lampa.followLineUntilDelayOrEncoder(125, 0, 0);        // cooldown
          debugLog("kamalt moftara9");

        } else if (i==1){
          lampa.setPIDNum(PID_0);
          lampa.setSpeed(120);
          lampa.followLineUntilDelayOrEncoder(200,0,0);
          lampa.followLineUntilSensor(SENSOR_RIGHT);
          lampa.followLineUntilDelayOrEncoder(0,150,150);
          // debugPause(1000);
          lampa.followLineUntilSensor(SENSOR_LEFT);

        }
        lampa.followLineUntilSensor(SENSOR_RIGHT); // dora el m3aftaa
        // lampa.followLineUntilSensor(0b10000000,0b00011000,XOR); //

        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 300, 0); // turn right
        lampa.setMotorUntilDelayOrEncoder(120, 120, 0, 30, 30); // 9addam chwaya
        // lampa.setMotorUntilDelayOrEncoder(-25, -25, 20, 0, 0); // debugggg
        lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 469);     // turn left
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL); // centri ro7k
        lampa.followLineUntilDelayOrEncoder(100, 0, 0);             // cooldown
        // lampa.followLineUntilDelayOrEncoder(1000, 0, 0);
        lampa.setPIDNum(PID_2);
        // lampa.followLineUntilSensor(0b10000000, 0b00011000, XOR); // detecti carro jary
        lampa.followLineUntilDelayOrEncoder(200,0,0);
        // debugPause(500);
        lampa.setPIDNum(PID_0);

        lampa.followLineUntilDelayOrEncoder(0, 80, 80); // centri ro7k
        lampa.setSpeed(180);
        lampa.setPIDNum(PID_1);
        // ejryyyyy fantar
        lampa.followLineUntilDelayOrEncoder(0, 250, 250); // encL:744 encR:771
        lampa.setSpeed(150);
        lampa.setPIDNum(PID_1);
        lampa.followLineUntilDelayOrEncoder(0, 200, 200); // encL:744 encR:771

        
        // raja3 low speed
        lampa.setSpeed(120);
        lampa.setPIDNum(PID_0);
        // lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR); // detecti carro w9ouf
        // encL:252 encR:1
        //  encL:2 encR:469

        // detecti dora m3afta 2
        lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR);
        lampa.forwardUntilDelayOrEncoder(0, 90, 90);
        debugPause(50);
        lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 375);
        // debugPause(50);
        lampa.forwardUntilDelayOrEncoder(0, 70, 70);
        // debugPause(50);
        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 550, 0);
        // debugPause(50);
        //  encL:94 encR:110
        //  encL:2 encR:260
        //  encL:71 encR:100
        //  encL:552 encR:0

        // kamalna dora lm3afta 2
      }
      if (1)
      {
        // ejri chwaya
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(150);
        lampa.followLineUntilDelayOrEncoder(0, 160, 160);
        lampa.followLineUntilSensor(SENSOR_RIGHT);

        // dour right
        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right
        lampa.setPIDNum(PID_1);
        lampa.setSpeed(180);
        lampa.followLineUntilDelayOrEncoder(0, 580, 580); // bdina theni dora right
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.followLineUntilDelayOrEncoder(0, 550, 0); // kamalna theni dora right
        lampa.setPIDNum(PID_1);
        lampa.setSpeed(180);
        lampa.followLineUntilDelayOrEncoder(0, 400, 0); // kamalna partie mestwia

        lampa.setPIDNum(PID_5);
        lampa.setSpeed(120);
        lampa.setPIDNum(PID_2);
        lampa.followLineUntilDelayOrEncoder(2600, 1500, 0); // kamalna nos dora s8ayra

        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL); // partie mestwia yesr twila

        //--------------------------------------- REMOVE ME
        unsigned int encL = get_encL(), encR = get_encR();
        unsigned int ms = millis();
        //--------------------------------------- REMOVE ME

        //--------------------------------------- REMOVE ME
        encL = get_encL() - encL;
        encR = get_encR() - encR;
        DebugSerial->println("encL: " + String(encL) + " encR: " + String(encR) + "ms: " + String(millis() - ms));
        //--------------------------------------- REMOVE ME

        // debugPause(1000);
        lampa.setPIDNum(PID_3);
        lampa.setSpeed(255);
        lampa.followLineUntilDelayOrEncoder(0, 500, 500); // kamalna partie mestwia
      }
      lampa.setPIDNum(PID_0);
      lampa.setSpeed(150);
      lampa.followLineUntilSensor(SENSOR_LEFT); // kamalna partie mestwia (carro loul)

      lampa.setPIDNum(PID_5);
      lampa.setSpeed(120);
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // kamalna partie mestwia (carro theni)
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // carro theleth
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // carro raba3 ba3d dora
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      // debugPause(1000);

      // lampa.followLineUntilSensor(SENSOR_LEFT);       // carro 5ames ba3d dora
      lampa.followLineUntilSensor(SENSOR_RIGHT_CENTER);       // carro 5ames ba3d dora
      // lampa.followLineUntilSensor(0b00000111,0b00011000,XOR);       // carro 5ames ba3d dora

      if (i==0){ //PITSTOP
      lampa.setMotorUntilDelayOrEncoder(120, -30, 0, 240, 0); // d5alna pitStop
      debugPause(1000);
      // lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
      lampa.followLineUntilSensor(SENSOR_EMPTY);
      lampa.forwardUntilDelayOrEncoder(0,50,50);
      lampa.setMotorUntilDelayOrEncoder(-30,-30,820,0,0);
      lampa.setMotorUntilDelayOrEncoder(-25,-25,1000,0,0);
      lampa.forwardUntilSensor(SENSOR_ALL);
      lampa.setPIDNum(PID_2);
      lampa.setSpeed(80);
      
      
      }
      else if (i==1 || i==2){
        // debugPause(1000);
        // encL:92 encR:80
        // encL:0 encR:157
        // encL:113 encR:1
        // --------------------
        // encL:66 encR:69
        // encL:58 encR:76
        lampa.setPIDNum(PID_2);
        lampa.setSpeed(80);
        lampa.forwardUntilDelayOrEncoder(0,64,64);
        debugPause(25);

        lampa.followLineUntilDelayOrEncoder(2000,0,0);
        // debugPause(500);
        // lampa.followLineUntilSensor(SENSOR_RIGHT);
        lampa.followLineUntilSensor(0b00011000,0b00000000,EQUAL);

        // lampa.setMotorUntilDelayOrEncoder(-120,120,0,58,58);
        
        
        // lampa.setMotorUntilDelayOrEncoder(120,120,0,80,80);
        // debugPause(25);
        // lampa.setMotorUntilDelayOrEncoder(120,-25,0,235,0);
        // debugPause(5000);

        // lampa.followLineUntilSensor(SENSOR_RIGHT);

        

        // debugPause(50);

      }
    }
    SerialController.update();
    lampa.ON = 0;
    DebugSerial->println("finished");
  }
  else if (lampa.ON == 2)
  { // code ytesti el pid w y5allik tbadlou bel bluetooth
    lampa.testPID();
    if (SerialController.update())
    {
      tunePIDCode();
    }
  }
  else if (lampa.ON == 0)
  { // code ywa9af e robot
    lampa.setMotor(0, 0);
    lampa.testPID();
    lampa.debugCode();
    if (SerialController.update())
    {
      tunePIDCode();
      if (lampa.ON == 2)
      {
        DebugSerial->println("testing Pid");
      }
    }
    delay(10);
  }
}