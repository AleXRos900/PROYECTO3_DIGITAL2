
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#define I2C_SDA 16
#define I2C_SCL 17



#define NUCLEO1_ADDR 0x55  // Nucleo 1
#define NUCLEO2_ADDR 0x60 // Nucleo 2

uint8_t ARREGLO_PARQUEOS = 0xFF;

const char* ssid     = "Jose";
const char* password = "quecojones";
WebServer server(80);

void setup() {
   Serial.begin(115200);
  delay(100);

  // 1) Configurar Wi-Fi como estación y desconectar todo estado previo
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  // 2) Iniciar conexión (UNA sola vez)
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // 3) Esperar a estar conectado, mostrando estado
  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Estado WiFi: %d (esperando…)\n", WiFi.status());
    delay(1000);
  }
  Serial.println("✓ Conectado a WiFi");
  Serial.print("IP asignada: ");
  Serial.println(WiFi.localIP());

  // 4) Ahora inicializar I2C y servidor HTTP
  Wire.begin(I2C_SDA, I2C_SCL);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/estado", HTTP_GET, handleEstado);
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  server.handleClient();
  // put your main code here, to run repeatedly:

  uint8_t BUFFER_NUCLEO_1 = 0;
  uint8_t BUFFER_NUCLEO_2 = 0;

  Wire.requestFrom(NUCLEO1_ADDR, 1);
  if (Wire.available()) BUFFER_NUCLEO_1 = Wire.read();

  Wire.requestFrom(NUCLEO2_ADDR, 1);
  if (Wire.available()) BUFFER_NUCLEO_2 = Wire.read();

  ARREGLO_PARQUEOS = (BUFFER_NUCLEO_1 & 0x0F) | (BUFFER_NUCLEO_2 & 0xF0);

  Wire.beginTransmission(NUCLEO1_ADDR);
  Wire.write(ARREGLO_PARQUEOS);
  Wire.endTransmission();

  delay(1500);

}

void handleRoot()
{
  String html;

html =  "<!DOCTYPE html>\n";
html += "<html lang=es>\n";
html += "<head>\n";
html += "<meta charset=UTF-8>\n";
html += "<meta name=viewport content=\"width=device-width, initial-scale=1\">\n";
html += "<title>Monitoreo de Parqueos</title>\n";
html += "<style>html,body{margin:0;padding:0;height:100%}body{background:url('https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/fondo.png') center/cover no-repeat}#wrapper{position:relative;width:100%;height:100vh}.overlay{position:absolute}#spot1{top:95px;left:1100px;width:115px;height:210px}#spot2{top:95px;left:1220px;width:115px;height:210px}#spot3{top:95px;left:1340px;width:115px;height:210px}#spot4{top:95px;left:1460px;width:115px;height:210px}#spot5{top:615px;left:1100px;width:115px;height:210px}#spot6{top:615px;left:1220px;width:115px;height:210px}#spot7{top:615px;left:1340px;width:115px;height:210px}#spot8{top:615px;left:1460px;width:115px;height:210px}#controls{position:absolute;top:50%;left:1rem;transform:translateY(-50%);background:rgba(255,255,255,0.9);padding:1rem;border-radius:8px;box-shadow:0 2px 6px rgba(0,0,0,0.3);font-family:Arial,sans-serif}#controls button{display:block;width:4rem;margin:.3rem 0;padding:.4rem;border:0;border-radius:4px;background:#3d85c6;color:#fff;cursor:pointer}#controls button.active{background:#f44336}</style>\n";
html += "</head>\n";
html += "<body>\n";
html += "<div id=wrapper>\n";
html += "<img id=spot1 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 1\">\n";
html += "<img id=spot2 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 2\">\n";
html += "<img id=spot3 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 3\">\n";
html += "<img id=spot4 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 4\">\n";
html += "<img id=spot5 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 5\">\n";
html += "<img id=spot6 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 6\">\n";
html += "<img id=spot7 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 7\">\n";
html += "<img id=spot8 class=overlay src=https://raw.githubusercontent.com/AleXRos900/ImagenePROYECTO3/refs/heads/main/CARROAZUL.png alt=\"Spot 8\">\n";
html += "<div id=controls>\n";
html += "<strong>Control Manual - PRUEBA</strong>\n";
html += "<button id=btn1 onclick=toggleSpot(1)>1</button>\n";
html += "<button id=btn2 onclick=toggleSpot(2)>2</button>\n";
html += "<button id=btn3 onclick=toggleSpot(3)>3</button>\n";
html += "<button id=btn4 onclick=toggleSpot(4)>4</button>\n";
html += "<button id=btn5 onclick=toggleSpot(5)>5</button>\n";
html += "<button id=btn6 onclick=toggleSpot(6)>6</button>\n";
html += "<button id=btn7 onclick=toggleSpot(7)>7</button>\n";
html += "<button id=btn8 onclick=toggleSpot(8)>8</button>\n";
html += "</div>\n";
html += "</div>\n";
html += "<script>const parking=[false,false,false,false,false,false,false,false];function updateSpots(){for(let i=0;i<parking.length;i++){const img=document.getElementById('spot'+(i+1));const btn=document.getElementById('btn'+(i+1));if(parking[i]){img.style.display='block';btn.classList.add('active');}else{img.style.display='none';btn.classList.remove('active');}}}\n";
"function toggleSpot(n){parking[n-1]=!parking[n-1];updateSpots();}\n";
"window.addEventListener('DOMContentLoaded',()=>{updateSpots();});</script>\n";
html += "</body>\n";
html += "</html>";

server.send(200, "text/html", html);
}

void handleEstado() {
  server.send(200, "text/plain", String(ARREGLO_PARQUEOS));
}
