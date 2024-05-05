//Définition de la broche du servomoteur
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
  
  const int servoPin1 = 11; // Broche de contrôle du servomoteur
  Servo servo1; // Création d'une instance du servomoteur

  const int servoPin2 = 10; // Broche de contrôle du servomoteur
  Servo servo2; // Création d'une instance du servomoteur

  const int servoPin3 = 6; // Broche de contrôle du servomoteur
  Servo servo3; // Création d'une instance du servomoteur

  const int servoPin4 = 5; // Broche de contrôle du servomoteur
  Servo servo4; // Création d'une instance du servomoteur

  //Définition des broches du détecteur de distance
  #define Broche_Echo 12
  #define Broche_Trigger 13

  int angle = 90;

  int mesureMax = 300;
  int mesureMin = 3;

  long duree;
  long distance;

      const int capteurPoidsPin = A5; // Broche analogique pour le capteur de poids
      const int seuilPoidsTropLourd=800;
  
  
  //Défintion des flags pour l'electroaimant
  bool flageElect = false;

  //Definition des flags de la marche du robot
  bool flagEnMarche = true;

  //Définition de la broche de l'electroaimant
  const int pinElec = 4;
  
  void setup() {
    
    //Définition des broches de sortie

    //Pour le servomoteur

    // On attache chaque servo à sa broche
    servo1.attach(servoPin1); 
    servo2.attach(servoPin2);
    servo3.attach(servoPin3);
    servo4.attach(servoPin4);
  
    servo1.write(0); // Faire tourner le servomoteur de 90 degrés
    servo2.write(0);
    servo3.write(0);
    servo4.write(0);

    //affichage des instructions sur l'écran LCD
   // Initialisation de l'écran LCD
  lcd.init();
  
  // Augmenter la luminosité du rétroéclairage
  lcd.setBacklight(255); // 255 représente la luminosité maximale
  
  // Affichage du message "Bonjour" sur l'écran LCD
  lcd.setCursor(0, 0); // Position du curseur en haut à gauche
    lcd.setCursor(0, 1); // Positionne le curseur à la deuxième ligne, première colonne
    lcd.print("Mets tes affaires dans le coffre"); // Affiche votre deuxième phrase sur la deuxième ligne
    delay(5000);
    lcd.clear();

    //Pin electroaimant
    pinMode(pinElec,OUTPUT);

    //vérification du poids
    
    int poids = analogRead(capteurPoidsPin); // Lire la valeur du capteur de poids
    while (poids > seuilPoidsTropLourd) {
    lcd.setCursor(0, 0);
    lcd.print("Poids trop lourd !");
    digitalWrite(pinElec,HIGH);
    
   }
   delay(1000);
    lcd.clear(); // Efface l'écran LCD
    lcd.setCursor(0, 0); // Positionne le curseur à la première ligne, première colonne
    lcd.print("Mon coffre  bien rempli");

   //On ferme l'electroaimant
    digitalWrite(pinElec,LOW);


    //On définit les broches du capteur de distance
    pinMode(Broche_Trigger, OUTPUT); // Broche Trigger en sortie //
    pinMode(Broche_Echo, INPUT); // Broche Echo en entree //
    Serial.begin (115200);

    

    
    

  }

  void loop() {

      if(flagEnMarche)
      {
        // Avancer
      for (angle = 90; angle <= 120; angle++)
       {
        servo1.write(angle);
        servo4.write(angle);
        delay(15);
      }
      delay(500);

      // Reculer
      for (angle = 120; angle >= 60; angle--) {
        servo1.write(angle);
        servo4.write(angle);
        delay(15);
      }
      delay(500);

      // Tourner à gauche
      for (angle = 90; angle >= 60; angle--) {
        servo2.write(angle);
        servo3.write(angle);
        delay(15);
      }
      delay(500);

      // Tourner à droite
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
