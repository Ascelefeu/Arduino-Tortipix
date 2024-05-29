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
  const int servoPin1 = 11; //Avant droit
  Servo servo1; 
  const int servoPin2 = 6; //Arrière gauche
  Servo servo2; 
  const int servoPin3 = 10; //Arrière droit
  Servo servo3;
  const int servoPin4 = 5; //Avant gauche
  Servo servo4; 

  //On instancie la borche du capteur de poids et le seuil
  const int capteurPoidsPin = A5; 
  const int seuilPoidsTropLourd=800;

  //Définition des broches du détecteur de distance
  #define Broche_Echo 12
  #define Broche_Trigger 13

  //Angle des servommoteurs
  int angleAvant = 0;
  int angleArriere = 0;

  //Variables capteur de distance
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

    //On initialise les servo à la position 0 ou 45 car on a inversé des servo sur le robot
    servo1.write(45);
    servo2.write(0);
    servo3.write(45);
    servo4.write(0);

     // Initialisation de l'écran LCD
    lcd.init();
    lcd.setBacklight(255);
    lcd.setCursor(0, 0); 

    //Instruction à l'utilisateur
    lcd.print("Remplie"); 
    lcd.setCursor(0, 1); 
    lcd.print("le coffre");
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
    lcd.print("Ma carapace ");
    lcd.setCursor(0,1);
    lcd.print("est remplie");
    digitalWrite(pinElec,HIGH);
    
   }

   

  void loop() {

    while(flagEnMarche)
    {

        // Pattes avant avancent
        for (angleAvant = 0; angleAvant <= 45; angleAvant++)
        {
          servo1.write(45-angleAvant);
          servo4.write(angleAvant);
          delay(15);
        }
        delay(500);

        // Pattes arrières avancent
        for (angleArriere = 0; angleArriere <= 45; angleArriere++) 
        {
          servo2.write(angleArriere);
          servo3.write(45-angleArriere);
          delay(15);
        }
        delay(500);

        // Toutes les pattes se replacent
        for (angleAvant = 45; angleAvant >= 0; angleAvant--) 
        {
          servo1.write(45-angleAvant);
          servo2.write(angleAvant);
          servo3.write(45-angleAvant);
          servo4.write(angleAvant);
          delay(15);
        }

        angleAvant = 0;
        angleArriere = 0;
        delay(500);

    

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
      }
      
    }
    digitalWrite(pinElec,HIGH);
    while(true)
    {}

  }
