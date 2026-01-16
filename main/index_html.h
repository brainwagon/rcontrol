#ifndef INDEX_HTML_H
#define INDEX_HTML_H

static const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121212; color: #e0e0e0; margin: 0; padding: 20px; text-align: center; }
  h1 { color: #bb86fc; }
  .container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; margin-bottom: 20px; }
  .box { background: #1e1e1e; padding: 20px; border-radius: 12px; width: 300px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
  h3 { margin-top: 0; border-bottom: 1px solid #333; padding-bottom: 10px; }
  
  /* Motor Bars */
  .motor-row { display: flex; align-items: center; margin: 15px 0; }
  .motor-label { width: 50px; text-align: right; margin-right: 10px; font-weight: bold; }
  .motor-track { flex-grow: 1; height: 24px; background: #333; border-radius: 12px; position: relative; overflow: hidden; }
  .motor-track::after { content: ''; position: absolute; left: 50%; top: 0; bottom: 0; width: 2px; background: #555; transform: translateX(-50%); z-index: 0; }
  .motor-fill { height: 100%; width: 0%; position: absolute; left: 50%; transition: all 0.1s ease; z-index: 1; }
  .motor-val { width: 80px; text-align: left; margin-left: 10px; font-family: monospace; font-size: 0.9em; }

  /* Bumpers */
  .bumper-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .bumper { padding: 15px; background: #333; border-radius: 8px; font-weight: bold; transition: background 0.2s; }
  .bumper.hit { background: #cf6679; color: #000; animation: shake 0.2s; }

  /* Lights */
  .light-row { display: flex; justify-content: space-around; margin-top: 20px; }
  .light { width: 40px; height: 40px; border-radius: 50%; background: #333; border: 2px solid #555; transition: all 0.2s; }
  .light.on { background: #ffeb3b; box-shadow: 0 0 15px #ffeb3b; border-color: #fff; }

  /* Status Indicator */
  .status-indicator { font-size: 1.2em; font-weight: bold; color: #cf6679; margin-top: 10px; }
  .status-indicator.connected { color: #00e676; }

  /* Logs */
  #log-window { 
    width: 95%; max-width: 800px; height: 300px; 
    background: #000; color: #00e676; 
    font-family: 'Consolas', 'Monaco', monospace; 
    overflow-y: scroll; text-align: left; 
    padding: 15px; margin: 0 auto; 
    border: 1px solid #333; border-radius: 5px; 
    font-size: 14px;
  }
  .log-line { margin: 2px 0; border-bottom: 1px solid #111; }
  
  @keyframes shake {
    0% { transform: translate(1px, 1px) rotate(0deg); }
    10% { transform: translate(-1px, -2px) rotate(-1deg); }
    20% { transform: translate(-3px, 0px) rotate(1deg); }
    30% { transform: translate(3px, 2px) rotate(0deg); }
    40% { transform: translate(1px, -1px) rotate(1deg); }
    50% { transform: translate(-1px, 2px) rotate(-1deg); }
    60% { transform: translate(-3px, 1px) rotate(0deg); }
    70% { transform: translate(3px, 1px) rotate(-1deg); }
    80% { transform: translate(-1px, -1px) rotate(1deg); }
    90% { transform: translate(1px, 2px) rotate(0deg); }
    100% { transform: translate(1px, -2px) rotate(-1deg); }
  }
</style>
</head>
<body>
  <h1>ESP32 Robot Dashboard</h1>
  
  <div class="container">
    <!-- Motor Status -->
    <div class="box">
      <h3>Motor Drive</h3>
      <div class="motor-row">
        <div class="motor-label">LEFT</div>
        <div class="motor-track">
          <div id="bar-ml" class="motor-fill"></div>
        </div>
        <div id="val-ml" class="motor-val">0</div>
      </div>
      <div class="motor-row">
        <div class="motor-label">RIGHT</div>
        <div class="motor-track">
          <div id="bar-mr" class="motor-fill"></div>
        </div>
        <div id="val-mr" class="motor-val">0</div>
      </div>
    </div>

    <!-- Bumper Status -->
    <div class="box">
      <h3>Bumper Sensors</h3>
      <div class="bumper-grid">
        <div id="b-fl" class="bumper">Front Left</div>
        <div id="b-fr" class="bumper">Front Right</div>
        <div id="b-rl" class="bumper">Rear Left</div>
        <div id="b-rr" class="bumper">Rear Right</div>
      </div>
    </div>

    <!-- Light Status -->
    <div class="box">
      <h3>Turn Signals</h3>
      <div class="light-row">
        <div>LEFT<br><div id="l-l" class="light"></div></div>
        <div>RIGHT<br><div id="l-r" class="light"></div></div>
      </div>
    </div>

    <!-- Controller Status -->
    <div class="box">
      <h3>Controller</h3>
      <div id="bt-status" class="status-indicator">Disconnected</div>
    </div>

    <!-- I2C Sensors -->
    <div class="box">
      <h3>I2C Sensors</h3>
      <div id="i2c-status" style="text-align:left; font-size:0.9em; font-family:monospace;">
        <div id="mpu-data" style="margin-bottom:5px; display:none;">
          <strong>MPU6050:</strong><br>
          Acc: <span id="mpu-ax">0</span>, <span id="mpu-ay">0</span>, <span id="mpu-az">0</span><br>
          Temp: <span id="mpu-temp">0</span> C
        </div>
        <div id="ina-data" style="margin-bottom:5px; display:none;">
          <strong>INA219:</strong> <span id="ina-status">Not Detected</span>
        </div>
        <div id="ssd-data" style="margin-bottom:5px; display:none;">
          <strong>SSD1306:</strong> <span id="ssd-status">Not Detected</span>
        </div>
        <div id="i2c-none" style="color:#777;">Scanning...</div>
      </div>
    </div>
  </div>

  <h3>System Logs</h3>
  <div id="log-window"></div>

  <script>
    var ws;
    function connect() {
      ws = new WebSocket('ws://' + location.host + '/ws');
      
      ws.onopen = function() {
        console.log('Connected');
        addLog('<span style="color:white">*** Connected to Robot ***</span>');
      };
      
      ws.onmessage = function(event) {
        try {
          var msg = JSON.parse(event.data);
          if (msg.type === 'status') {
            updateStatus(msg);
          } else if (msg.type === 'log') {
            addLog(msg.data);
          }
        } catch(e) {
          console.error(e);
        }
      };

      ws.onclose = function() {
        console.log('Closed');
        addLog('<span style="color:red">*** Connection Lost. Reconnecting... ***</span>');
        setTimeout(connect, 2000);
      };
    }

    function updateStatus(d) {
        setMotor('ml', d.ml);
        setMotor('mr', d.mr);
        
        setBumper('fl', d.b[0]);
        setBumper('fr', d.b[1]);
        setBumper('rl', d.b[2]);
        setBumper('rr', d.b[3]);
        
        setLed('l', d.ll);
        setLed('r', d.lr);
        
        setBt(d.bt);

        if (d.i2c) updateI2C(d.i2c);
    }

    function updateI2C(i2c) {
        var hasDev = false;
        if (i2c.mpu6050) {
            document.getElementById('mpu-data').style.display = 'block';
            document.getElementById('mpu-ax').innerText = i2c.mpu6050.ax;
            document.getElementById('mpu-ay').innerText = i2c.mpu6050.ay;
            document.getElementById('mpu-az').innerText = i2c.mpu6050.az;
            document.getElementById('mpu-temp').innerText = i2c.mpu6050.temp.toFixed(1);
            hasDev = true;
        }
        if (i2c.ina219) {
            document.getElementById('ina-data').style.display = 'block';
            document.getElementById('ina-status').innerText = i2c.ina219.status;
            hasDev = true;
        }
        if (i2c.ssd1306) {
            document.getElementById('ssd-data').style.display = 'block';
            document.getElementById('ssd-status').innerText = i2c.ssd1306;
            hasDev = true;
        }
        
        document.getElementById('i2c-none').style.display = hasDev ? 'none' : 'block';
    }

    function setBt(connected) {
        var el = document.getElementById('bt-status');
        if (connected) {
            el.innerText = "Connected";
            el.classList.add('connected');
        } else {
            el.innerText = "Disconnected";
            el.classList.remove('connected');
        }
    }

    function setMotor(id, val) {
        var text = "STOP";
        if (val > 0) text = "FWD " + val;
        else if (val < 0) text = "REV " + Math.abs(val);
        
        document.getElementById('val-'+id).innerText = text;
        var bar = document.getElementById('bar-'+id);
        var w = Math.abs(val) / 2; 
        
        bar.style.width = w + '%';
        // Center is 50%. 
        // If val > 0 (forward), left is 50%.
        // If val < 0 (reverse), left is 50% - w%.
        bar.style.left = (val < 0 ? (50 - w) : 50) + '%';
        bar.style.backgroundColor = val < 0 ? '#ff5252' : '#03dac6';
    }

    function setBumper(id, hit) {
        var el = document.getElementById('b-'+id);
        if (hit) el.classList.add('hit');
        else el.classList.remove('hit');
    }

    function setLed(id, on) {
        var el = document.getElementById('l-'+id);
        if (on) el.classList.add('on');
        else el.classList.remove('on');
    }

    function addLog(txt) {
        var win = document.getElementById('log-window');
        // Simple timestamp
        var d = new Date();
        var time = d.getHours() + ":" + d.getMinutes() + ":" + d.getSeconds() + "." + d.getMilliseconds();
        
        win.innerHTML += '<div class="log-line">[' + time + '] ' + txt + '</div>';
        win.scrollTop = win.scrollHeight;
        
        // Limit log lines to prevent browser crash
        if (win.childElementCount > 200) {
            win.removeChild(win.firstElementChild);
        }
    }

    connect();
  </script>
</body>
</html>
)rawliteral";

#endif
