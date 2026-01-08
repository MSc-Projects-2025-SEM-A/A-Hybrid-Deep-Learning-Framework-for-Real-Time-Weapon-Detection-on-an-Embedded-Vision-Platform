#include <WiFi.h>
#include <WebServer.h>

// -------------------------
// Ultrasonic Pins
// -------------------------
#define TRIGF 5
#define ECHOF 18
#define TRIGL 16
#define ECHOL 17
#define TRIGR 19
#define ECHOR 21

// -------------------------
// Motor Driver Pins (L298N)
// -------------------------
#define ENA 23
#define IN1 25
#define IN2 26
#define ENB 22
#define IN3 27
#define IN4 14

// -------------------------
// Q-Learning Parameters
// -------------------------
float Q[27][4];
float alpha = 0.3;       // learning rate
float discount = 0.8;    // renamed gamma → discount
float epsilon = 0.2;     // exploration rate

// -------------------------
// Grid Map Settings
// -------------------------
const int GRID_SIZE = 41;
const int CENTER = GRID_SIZE/2;
const int MAX_STEPS = 2000;

int grid[GRID_SIZE][GRID_SIZE];
int pathX[MAX_STEPS];
int pathY[MAX_STEPS];
int pathLen = 0;

// Robot Estimated Pose
int robotX = CENTER;
int robotY = CENTER;
int robotDir = 0; // 0=N,1=E,2=S,3=W

// Movement timing (tune these)
const int FORWARD_MS = 300;
const int TURN_MS = 220;

// -------------------------
// WebServer
// -------------------------
WebServer server(80);

// -------------------------
// Motors
// -------------------------
void motorForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void motorBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void motorLeftTurn() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void motorRightTurn() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// -------------------------
// Ultrasonic
// -------------------------
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  long dist = duration * 0.034 / 2;
  if (dist == 0) dist = 300;
  return dist;
}

// -------------------------
// Grid & Path Logging
// -------------------------
void recordPosition() {
  if (pathLen < MAX_STEPS) {
    pathX[pathLen] = robotX;
    pathY[pathLen] = robotY;
    pathLen++;
  }
  grid[robotY][robotX]++;
}

void moveForwardGrid() {
  motorForward();
  delay(FORWARD_MS);
  motorStop();

  if (robotDir == 0 && robotY > 0) robotY--;
  else if (robotDir == 1 && robotX < GRID_SIZE-1) robotX++;
  else if (robotDir == 2 && robotY < GRID_SIZE-1) robotY++;
  else if (robotDir == 3 && robotX > 0) robotX--;

  recordPosition();
}
void turnLeftGrid() {
  motorLeftTurn();
  delay(TURN_MS);
  motorStop();

  robotDir = (robotDir + 3) % 4;
}
void turnRightGrid() {
  motorRightTurn();
  delay(TURN_MS);
  motorStop();

  robotDir = (robotDir + 1) % 4;
}
void backGrid() {
  motorBackward();
  delay(FORWARD_MS);
  motorStop();

  if (robotDir == 0 && robotY < GRID_SIZE-1) robotY++;
  else if (robotDir == 1 && robotX > 0) robotX--;
  else if (robotDir == 2 && robotY > 0) robotY--;
  else if (robotDir == 3 && robotX < GRID_SIZE-1) robotX++;

  recordPosition();
}

// -------------------------
// State + Reward
// -------------------------
int getState() {
  long F = getDistance(TRIGF, ECHOF);
  long L = getDistance(TRIGL, ECHOL);
  long R = getDistance(TRIGR, ECHOR);

  int Fz = (F < 20 ? 0 : (F < 40 ? 1 : 2));
  int Lz = (L < 20 ? 0 : (L < 40 ? 1 : 2));
  int Rz = (R < 20 ? 0 : (R < 40 ? 1 : 2));

  return Fz*9 + Lz*3 + Rz;
}

int getReward(int state) {
  int Fz = state / 9;
  if (Fz == 0) return -20;
  if (Fz == 1) return +5;
  if (Fz == 2) return +10;
  return 0;
}

// -------------------------
// Q-Learning Action
// -------------------------
int chooseAction(int state) {
  if ((float)random(0,1000)/1000.0 < epsilon) {
    return random(0,4);
  }
  int best = 0;
  float bestVal = Q[state][0];
  for (int a=1; a<4; a++) {
    if (Q[state][a] > bestVal) {
      best = a;
      bestVal = Q[state][a];
    }
  }
  return best;
}

// -------------------------
// Web UI JSON
// -------------------------
String getMapJSON() {
  String s = "{";
  s += "\"gridSize\":" + String(GRID_SIZE) + ",";
  s += "\"pathLen\":" + String(pathLen) + ",";
  s += "\"path\":[";

  for (int i=0; i<pathLen; i++) {
    s += "{\"x\":" + String(pathX[i]) + ",\"y\":" + String(pathY[i]) + "}";
    if (i < pathLen-1) s += ",";
  }
  s += "],\"visits\":[";

  bool first = true;
  for (int y=0; y<GRID_SIZE; y++) {
    for (int x=0; x<GRID_SIZE; x++) {
      if (grid[y][x] > 0) {
        if (!first) s += ",";
        first = false;
        s += "{\"x\":" + String(x) + ",\"y\":" + String(y) + ",\"v\":" + String(grid[y][x]) + "}";
      }
    }
  }
  s += "]}";
  return s;
}

void handleRoot() {
  String html = R"====(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 AI Patrol Map</title>
<style>
body { font-family: Arial; }
canvas { border:1px solid #000; }
</style>
</head>
<body>
<h2>AI Patrol Path Map</h2>
<canvas id="map" width="512" height="512"></canvas>
<script>
async function update(){
  let res = await fetch('/map');
  let j = await res.json();
  draw(j);
}
function draw(j){
  let c = document.getElementById("map");
  let ctx = c.getContext("2d");
  ctx.fillStyle="white";
  ctx.fillRect(0,0,512,512);
  let cell = 512/j.gridSize;

  // visits
  j.visits.forEach(v=>{
    ctx.fillStyle=`rgba(0,150,255,${Math.min(1,v.v*0.1)})`;
    ctx.fillRect(v.x*cell, v.y*cell, cell, cell);
  });

  // path
  ctx.strokeStyle="red";
  ctx.lineWidth=2;
  ctx.beginPath();
  if(j.pathLen>0){
    ctx.moveTo((j.path[0].x+0.5)*cell,(j.path[0].y+0.5)*cell);
    for(let i=1;i<j.pathLen;i++){
      ctx.lineTo((j.path[i].x+0.5)*cell,(j.path[i].y+0.5)*cell);
    }
    ctx.stroke();
  }
}
setInterval(update,800);
update();
</script>
</body>
</html>
)====";
  server.send(200, "text/html", html);
}

void handleMap() {
  server.send(200, "application/json", getMapJSON());
}

// -------------------------
// Setup
// -------------------------
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(34));

  pinMode(TRIGF,OUTPUT); pinMode(ECHOF,INPUT);
  pinMode(TRIGL,OUTPUT); pinMode(ECHOL,INPUT);
  pinMode(TRIGR,OUTPUT); pinMode(ECHOR,INPUT);

  pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  // init map
  for(int i=0;i<GRID_SIZE;i++)
    for(int j=0;j<GRID_SIZE;j++)
      grid[i][j]=0;

  recordPosition();

  // init Q table
  for(int s=0;s<27;s++)
    for(int a=0;a<4;a++)
      Q[s][a] = 0;

  // WiFi AP mode
  WiFi.softAP("PatrolBotAP");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/map", handleMap);
  server.begin();
}

// -------------------------
// MAIN LOOP
// -------------------------
void loop() {
  server.handleClient();

  int state = getState();
  int action = chooseAction(state);

  switch(action){
    case 0: moveForwardGrid(); break;
    case 1: turnLeftGrid(); break;
    case 2: turnRightGrid(); break;
    case 3: backGrid(); break;
  }

  int reward = getReward(getState());
  int newState = getState();

  float maxQ = Q[newState][0];
  for (int a=1;a<4;a++)
    if (Q[newState][a] > maxQ)
      maxQ = Q[newState][a];

  Q[state][action] =
    (1 - alpha)*Q[state][action] +
    alpha*(reward + discount * maxQ);

  Serial.print("S:"); Serial.print(state);
  Serial.print(" A:"); Serial.print(action);
  Serial.print(" R:"); Serial.println(reward);

  delay(120);
}
