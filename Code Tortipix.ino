  //On inclue les bibliothèques servomoteur/écran lcd
  #include <Servo.h>
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  //Définir l'adresse I2C de l'ecran
  #define LCD_ADDR 0x27

  //Définir les dimensions de l'ecran LCD
  #define LCD_COLUMNS 16
  #define LCD_ROWS 2

  //Créer une instance de l'écran LCD
  LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);

  //On définit les broches des quatres servomoteurs
  const int servoPin1 = 11;
  Servo servo1; 
  const int servoPin2 = 10;
  Servo servo2; 
  const int servoPin3 = 6; 
  Servo servo3;
  const int servoPin4 = 5; 
  Servo servo4; 

  //On instancie la borche du capteur de poids et le seuil
  const int capteurPoidsPin = A5; 
  const int seuilPoidsTropLourd=800;

  //Définition des broches du détecteur de distance
  #define Broche_Echo 12
  #define Broche_Trigger 13

  int angle = 90;
  int mesureMax = 300;
  int mesureMin = 3;
  long duree;
  long distance;

  //Défintion des flags pour l'electroaimant
  bool flageElect = false;

  //Definition des flags de la marche du robot
  bool flagEnMarche = true;

  //Définition de la broche de l'electroaimant
  const int pinElec = 4;
  
  void setup() {
    
    // On attache chaque servo à sa broche
    servo1.attach(servoPin1); 
    servo2.attach(servoPin2);
    servo3.attach(servoPin3);
    servo4.attach(servoPin4);

    //On initialise les servo à la position 0
    servo1.write(0);
    servo2.write(0);
    servo3.write(0);
    servo4.write(0);

    // Initialisation de l'écran LCD
    lcd.init();
    lcd.setBacklight(255);
    lcd.setCursor(0, 0); 
    lcd.setCursor(0, 1); 

    //Instruction à l'utilisateur
    lcd.print("Mets tes affaires dans le coffre"); 
    delay(5000);
    lcd.clear();

    //Pin electroaimant
    pinMode(pinElec,OUTPUT);

    //On définit les broches du capteur de distance
    pinMode(Broche_Trigger, OUTPUT); 
    pinMode(Broche_Echo, INPUT); 
    Serial.begin (115200);

    digitalWrite(pinElec,HIGH);

    //vérification du poids
    int poids = analogRead(capteurPoidsPin); 
    while (poids > seuilPoidsTropLourd) {
    lcd.setCursor(0, 0);
    lcd.print("Poids trop lourd !");
    poids = analogRead(capteurPoidsPin); 
    delay(1500);
    }

    //Une fois le coffre rempli, on "ferme" l'electroaimant
    delay(1000);
    lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("Ma carapace est bien remplie !");
    digitalWrite(pinElec,LOW);
    
   }

  void loop() {

      //Tant qu'il n'y a pas d'obstacle, on avance
      //Les pattes font un mouvements de vas et viens 2 à deux
      if(flagEnMarche)
      {
        // Pattes avant avancent
      for (angle = 90; angle <= 120; angle++)
       {
        servo1.write(angle);
        servo4.write(angle);
        delay(15);
      }
      delay(500);

      // Pattes avant reculent
      for (angle = 120; angle >= 60; angle--) {
        servo1.write(angle);
        servo4.write(angle);
        delay(15);
      }
      delay(500);

      // Pattes arrières avancent
      for (angle = 90; angle >= 60; angle--) {
        servo2.write(angle);
        servo3.write(angle);
        delay(15);
      }
      delay(500);

      // Pattes arrières reculent
      for (angle = 60; angle <= 90; angle++) {
        servo2.write(angle);
        servo3.write(angle);
        delay(15);
      }
    delay(500);
    //mesure de la distance à l'aide du capteur
      
      // Debut de la mesure avec un signal de 10 μS applique sur TRIG //
      digitalWrite(Broche_Trigger, LOW); // On efface l'etat logique de TRIG //
      delayMicroseconds(2);
      digitalWrite(Broche_Trigger, HIGH); // On met la broche TRIG a "1" pendant 10μS //
      delayMicroseconds(10);
      digitalWrite(Broche_Trigger, LOW); // On remet la broche TRIG a "0" //
      
      // On mesure combien de temps le niveau logique haut est actif sur ECHO //
      duree = pulseIn(Broche_Echo, HIGH);
      
      // Calcul de la distance grace au temps mesure //
      distance = duree*0.034/2; 
      
      // Verification si valeur mesuree dans la plage //
      if (distance >= mesureMax || distance <= mesureMin ||  analogRead(capteurPoidsPin) > seuilPoidsTropLourd) {
      flagEnMarche = false;
      // Si la distance est hors plage, on affiche un message d'erreur //
      //Serial.println("Distance de mesure en dehors de la plage (3 cm à 3 m)");
      }
      }

      //
    
    //else {
    // Affichage dans le moniteur serie de la distance mesuree //
    //Serial.print("Distance mesuree :");
    //Serial.print(distance);
    //Serial.println("cm");}
      
    
    //while(true)
    //{}

  }
