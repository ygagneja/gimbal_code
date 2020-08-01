import processing.serial.*;

Serial mySerial;

String myString = null;
int nl = 10;

float x, y, z;
float a, b, c;

void setup() {
   frameRate(120);
   mySerial = new Serial(this, "/dev/ttyUSB0", 115200);
   size(700, 700, P3D);
   x = -0.08 - PI/2;
   y = -0.08 - PI/2;
   z = -PI/2;
   a = 90;
   b = 90;
   c = 90;
}

void draw() {
   
     myString = mySerial.readStringUntil(nl);
     if(myString != null){
       int[] nums = int(split(myString, '\t'));
       a = nums[0];
       b = nums[1];
       c = nums[2];
       background(0);
       strokeWeight(2);
       stroke(255);
       translate(width/2, height/2, 0);
       line(0, -300, 0, 300);
       line(-300, 0, 300, 0);
       fill(255);
       textSize(32);
       text("X",320,12);
       text("Y",-8,-320);
       text("X : " + str(c), 200, -300);
       text("Y : " + str(a), 200, -260);
       text("Z : " + str(b), 200, -220);
       //line(-200, 200, 200, -200);
       rotateX(x + PI*c/180);
       rotateY(y - PI*a/180);
       rotateZ(z + PI*b/180);
       fill(250,150,50);
       stroke(0,255,0);
       box(200);
       translate(100, 0, 0);
       noStroke();
       fill(#ff0000);
       lights();
       sphere(20);
       translate(-100, -100, 0);
       sphere(20);
       translate(0, 100, 100);
       sphere(20);
   }
}
