import controlP5.*;
import processing.serial.*;
Serial myPort;
String myString="";
ControlP5 cp5 ;
PFont font;
void setup() {
  cp5 = new ControlP5(this);
  
    font = createFont("calibri light bold", 20);    // custom fonts for buttons and title
  
  cp5.addButton("w")     //"" is the name of button
    .setPosition(180, 0)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
    font = createFont("calibri light bold", 20);    // custom fonts for buttons and title
  
  cp5.addButton("s")     //"" is the name of button
    .setPosition(180, 150)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
    font = createFont("calibri light bold", 20);    // custom fonts for buttons and title
  
  cp5.addButton("a")     //"" is the name of button
    .setPosition(50, 75)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
    font = createFont("calibri light bold", 20);    // custom fonts for buttons and title
  
  cp5.addButton("d")     //"" is the name of button
    .setPosition(310, 75)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
    cp5.addButton("X")     //"" is the name of button
    .setPosition(180, 75)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("q")     //"" is the name of button
    .setPosition(50, 0)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("z")     //"" is the name of button
    .setPosition(50, 150)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("e")     //"" is the name of button
    .setPosition(310, 0)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("c")     //"" is the name of button
    .setPosition(310, 150)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("j")     //"" is the name of button
    .setPosition(500, 75)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
      cp5.addButton("k")     //"" is the name of button
    .setPosition(600, 75)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .setFont(font)
  ; 
 
  // List all the available serial ports
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[2], 115200);
  myPort.bufferUntil('\n') ;
  size(750, 210);
  noSmooth();
  background(0);
  translate(410, 410);
  stroke(255);
  strokeWeight(3);
}
void draw() {
  background(0, 0 , 0);
  if (keyPressed) {
    if (key == 'w' || key == 'W') {
      myPort.write(119) ;
    }
    if (key == 's' || key == 'S') {
      myPort.write('s') ;
    }
    if (key == 'a' || key == 'A') {
      myPort.write('a') ;
    }
    if (key == 'd' || key == 'D') {
      myPort.write('d') ;
    }
    if (key == 'x' || key == 'X') {
      myPort.write('x') ;
    }
    if (key == 'j' || key == 'J') {
      myPort.write('j') ;
    }
    if (key == 'k' || key == 'K') {
      myPort.write('k') ;
    }
    if (key == 'q' || key == 'Q') {
      myPort.write('q') ;
    }
    if (key == 'e' || key == 'E') {
      myPort.write('e') ;
    }
    if (key == 'z' || key == 'Z') {
      myPort.write('z') ;
    }
    if (key == 'c' || key == 'C') {
      myPort.write('c') ;
    }
    
    myPort.clear();
}
}

void w(){
  myPort.write('w') ;
}
void s(){
  myPort.write('s') ;
}
void a(){
  myPort.write('a') ;
}
void d(){
  myPort.write('d') ;
}
void X(){
  myPort.write('x') ;
}
void c(){
  myPort.write('c') ;
}
void z(){
  myPort.write('z') ;
}
void e(){
  myPort.write('e') ;
}
void q(){
  myPort.write('q') ;
}
void j(){
  myPort.write('j') ;
}
void k(){
  myPort.write('k') ;
}
