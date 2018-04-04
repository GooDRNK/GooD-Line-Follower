#include <QTRSensors.h> //Libraria necesara pentru senzori.
#define RDF 6 //Roata Dreapta Fata
#define RDS 9 //Roata Dreapta Spate
#define RSF 3 //Roata Stanga Fata
#define RSS 5 //Roata Stanga Spate
#define Speed 100 //Viteza maxima in linie dreapta
#define SpTurn 150 //Viteza de rotire la 90 de grade.
#define Frana 50 //Frana in linie dreapta aplicata la un interval de secunde.
#define DF 70 //Viteza necesara pe ambele motoare pentru aplicarea franei.
#define Kp 10 //Constanta pentru PID(Proportionala). (Nu copiati aceste valori identic, aceste valori se gasesc prin teste pentru fiecare robot.)
#define Kd 55 //Constanta pentru PID(Derivata). (Nu copiati aceste valori identic, aceste valori se gasesc prin teste pentru fiecare robot.)
#define nrlinipunctate 4 //Numarul de linii punctate regasite pe traseu.
QTRSensorsRC Snz((unsigned char[]) {14, 15, 16, 17, 18, 19, 8}, 7, 1000, 2); //Declarare senzori cu libraria QTR.
unsigned int SV[7]; //Variabila in care se stocheaza intensitatea senzorului pe fiecare dioda.
int PID; //Valoarea returnata de PID pentru corectia motoarelor.
int le; //ultima eroare
int err; //eroare fata de linie
int i //contor pentru linia punctata
boolean pct = false; //variabila ce arata daca a fost terminata linia punctata pe traseu 
int lpos; //ultima pozitie salvata
int si[4]; //Valuarea in digital a unui senzor.
int sl[4]; //Ultima valuare salvata in digital a unui senzor.
int franac=0; //Contor de la ultima frana pusa in linie dreapta.
void loop() 
{
  //Citire//
  int pos = Snz.readLine(SV); 
  si[0]=digitalRead(14);
  si[1]=digitalRead(15);
  si[2]=digitalRead(8);
  si[3]=digitalRead(19);
  //Detectia liniei punctate
  if ((pos==0 || pos==6000) && lpos>1700 && lpos<4300 && sl[0]==0 && sl[1]==0 && sl[2]==0 && sl[3]==0 && pct==false)
  {
        while (i <= nrlinipunctate)
        {
          while (Snz.readLine(SV) == 0 || Snz.readLine(SV) == 6000) 
          {
            G(60, 60);
          }
          while (Snz.readLine(SV) != 0 && Snz.readLine(SV) != 6000) 
          {
            D(Snz.readLine(SV));
          }
          i++;
        } 
        pct = true;
        return;
   }
  //Detectare unghi de 90 pe partea stanga.
  if (Snz.readLine(SV) <= 100) 
  {
    G(-SpTurn, SpTurn);franac=0; 
    return;
  }
  //Detectare ungi de 90 pe partea dreapta.
  if (Snz.readLine(SV) >= 5900) 
  {
    G(SpTurn, -SpTurn);franac=0; 
    return;
  }
  //Memorarea ultimelor variabile folosite Pozitia/Senzorii
  lpos=pos;
  sl[0]=si[0];
  sl[1]=si[1];
  sl[2]=si[2];
  s1[3]=si[3];
  //
  D(Snz.readLine(SV));
}
void setup() 
{
  //Declararea pinilor pentru motoare
  pinMode(RDF, OUTPUT);
  pinMode(RDS, OUTPUT);
  pinMode(RSF, OUTPUT);
  pinMode(RSS, OUTPUT);
  //Declararea frecventei pentru serial port
  Serial.begin(9600);
  //Auto calibrarea necesara pentru senzori.
  for (int i = 0; i <= 250; i++) 
  {
    Snz.calibrate();
  }
  Snz.readLine(SV);
}
//Functia PID combinata cu frana.
void D(int p) {
  int rMS, lMS;
  err = (p / 100) - 30;
  PID = (Kp * err) + (Kd * (err - le));
  le = err;
  if (PID > Speed) 
  {
    PID = Speed;
  } if (PID < -1 * Speed) 
  {
    PID = -1 * Speed;
  } if (PID < 0) 
  {
    rMS = Speed;
    lMS = Speed - abs(int(PID));
  } else 
  {
    rMS = Speed - abs(int(PID));
    lMS = Speed;
  } 
  if (rMS > DF && lMS > DF) 
  {
    franac++;
    if(franac>300)
    {
     G(Frana, Frana);
     franac=0; 
     return;
    }
  }
  G(lMS, rMS);
}
//Controlul motoarelor
void G(int sL, int sR) {
  if (sL > 0) {
    digitalWrite(RSF, HIGH);
    digitalWrite(RSS, LOW);
    analogWrite(RSF, sL);
    analogWrite(RSS, 0);
  } else {
    digitalWrite(RSF, LOW);
    digitalWrite(RSS, HIGH);
    analogWrite(RSF, 0);
    analogWrite(RSS, -sL);
  } if (sR > 0) {
    digitalWrite(RDF, HIGH);
    digitalWrite(RDS, LOW);
    analogWrite(RDF, sR);
    analogWrite(RDS, 0);
  } else {
    digitalWrite(RDF, LOW);
    digitalWrite(RDS, HIGH);
    analogWrite(RDF, 0);
    analogWrite(RDS, -sR);
  }
}
