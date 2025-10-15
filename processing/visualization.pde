import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;
String data = "";
float roll, pitch, yaw;

// Modelo 3D
PShape model;

void setup() {
  size(800, 600, P3D);

  // Carregar o modelo 3D (coloque "meu_objeto.obj" dentro da pasta "data")
  model = loadShape("meu_objeto2.obj");

  // Escalar o modelo, se necessário
  // ajuste conforme necessário
  model.scale(10); 

  // Iniciar comunicação serial
  println("Portas seriais disponíveis:");
  String[] portas = Serial.list();
  for (int i = 0; i < portas.length; i++) {
    println(i + ": " + portas[i]);
  }
  int indicePorta = 0; // escolha o índice correto
  myPort = new Serial(this, portas[indicePorta], 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(0);
  lights();
  translate(width/2, height/2, 0);
  
  textSize(22);
  fill(255);
  text("Roll: " + int(roll) + "     Pitch: " + int(pitch) + "     Yaw: " + int(yaw), -100, 265);
  
  if (!Float.isNaN(pitch) && !Float.isNaN(roll) && !Float.isNaN(yaw)) {
    // Rotacionar o modelo conforme os ângulos recebidos
    rotateX(radians(pitch));
    rotateY(radians(roll));
    rotateZ(radians(yaw));
  }
  
  // Desenhar o modelo
  shape(model);
  
  // Texto 3D dentro da cena (opcional)
  fill(255);
  textSize(25);
}

// Leitura dos dados seriais
void serialEvent(Serial myPort) { 
  data = myPort.readStringUntil('\n');
  if (data != null) {
    data = trim(data);
    println(data);
    String items[] = split(data, ',');
    if (items.length > 2) {
      roll = float(items[0]);
      pitch = float(items[1]);
      yaw = float(items[2]);
    }
  }
}
