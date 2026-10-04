#include <WebServer.h>
#include <WiFi.h>
#include "common.h"

static WebServer server(80);

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TM Robot</title>
<style>
body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:0;background:#101820;color:#eef4f7}
main{max-width:980px;margin:auto;padding:18px}
section{border:1px solid #304452;border-radius:8px;padding:14px;margin:12px 0;background:#16232d}
h1{font-size:24px;margin:0 0 10px}h2{font-size:17px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
.metric{background:#0c141b;border:1px solid #263743;border-radius:6px;padding:10px}
.metric b{display:block;color:#8fc7ff;font-size:13px}.metric span{font-size:22px}
button{border:0;border-radius:6px;padding:10px 12px;margin:4px;background:#2f80ed;color:white;font-weight:700;cursor:pointer}
button.stop{background:#d64545}button.reset{background:#75808a}button.cal{background:#16a085}
button:disabled{opacity:.4;cursor:default}input,select{width:90px;padding:8px;border-radius:6px;border:1px solid #506574;background:#0c141b;color:#eef4f7}
label{display:inline-flex;align-items:center;gap:8px;margin:4px 12px 4px 0}
pre{white-space:pre-wrap;max-height:300px;overflow:auto;background:#05090d;border:1px solid #263743;border-radius:6px;padding:10px}
.row{display:flex;flex-wrap:wrap;align-items:center;gap:8px}.fault{color:#ffb1b1}.ok{color:#a4e8b5}
#error{color:#ffb1b1;margin:8px 4px 0}#error:empty{display:none}
</style>
</head>
<body><main>
<h1>TM Robot Control</h1>
<section><h2>State</h2>
<div class="grid">
<div class="metric"><b>State</b><span id="state">Connecting...</span></div>
<div class="metric"><b>Fault</b><span id="fault">-</span></div>
<div class="metric"><b>Calibration</b><span id="calibration">-</span></div>
</div>
<div class="row">
<button id="arm" data-command="arm" disabled>Arm</button>
<button class="stop" data-command="stop">Stop</button>
<button class="reset" id="reset" data-command="reset" disabled>Reset fault</button>
<button class="cal" id="calibrate" disabled>Calibrate on rear panel</button>
</div>
<p>Rest motionless on the rear panel on a level surface to calibrate, then stand upright before arming.</p>
<p id="error" role="status"></p>
</section>
<section>
<div class="grid">
<div class="metric"><b>Pitch</b><span id="pitch">-</span></div>
<div class="metric"><b>Angular rate</b><span id="rate">-</span></div>
<div class="metric"><b>Torque request</b><span id="tau">-</span></div>
<div class="metric"><b>Tilt bias</b><span id="bias">-</span></div>
</div></section>
<section><h2>PID</h2>
<p id="activeGains">-</p>
<form id="gains">
<label>Kp <input name="kp" id="kp" type="number" min="0" max="20" step="any" required></label>
<label>Ki <input name="ki" id="ki" type="number" min="0" max="10" step="any" required></label>
<label>Kd <input name="kd" id="kd" type="number" min="0" max="10" step="any" required></label><br>
<button type="submit" value="0">Apply</button>
<button type="submit" value="1" id="saveGains" disabled>Apply and save</button>
</form>
</section>
<section><h2>IMU orientation</h2>
<p id="activeAxis">-</p>
<form id="mounting">
<label>Axis <select id="axis" name="axis"><option value="x">X / roll</option><option value="y">Y / pitch</option></select></label>
<label>Sign <select id="sign" name="sign"><option value="-1">-1</option><option value="1">+1</option></select></label><br>
<button type="submit" value="0" id="applyAxis" disabled>Apply</button>
<button type="submit" value="1" id="saveAxis" disabled>Apply and save</button>
</form>
</section>
<section><h2>Logs</h2><pre id="messages"></pre></section>
</main>
<script>
const el=id=>document.getElementById(id);
let first=true, reading=false;
async function status(){
 if(reading)return;
 reading=true;
 try{
  const response=await fetch('/api/status',{cache:'no-store'});
  if(!response.ok)throw new Error('Could not read robot status');
  const s=await response.json();
  el('state').textContent=s.state;el('fault').textContent=s.fault;
  for(const [id,value,unit] of [['pitch',s.pitch,'rad'],['rate',s.rate,'rad/s'],['tau',s.tau,'Nm'],['bias',s.bias,'rad']])
   el(id).textContent=Number(value).toFixed(3)+' '+unit;
  el('calibration').textContent=s.calibration==='idle'&&!s.calibrated?'required':s.calibration;
  el('activeGains').textContent=`Active: Kp ${s.kp}, Ki ${s.ki}, Kd ${s.kd}`;
  el('activeAxis').textContent=`Active: ${s.axis.toUpperCase()}, sign ${s.sign}`;
  el('messages').textContent=s.messages;
  const stopped=s.state==='DISARMED'&&s.calibration==='idle';
  el('arm').disabled=!stopped||!s.calibrated;el('calibrate').disabled=!stopped;
  el('reset').disabled=s.state!=='FAULT';
  for(const id of ['saveGains','applyAxis','saveAxis'])el(id).disabled=!stopped;
  if(first){for(const id of ['kp','ki','kd','axis','sign'])el(id).value=s[id];first=false;}
  if(el('error').dataset.connection==='1'){el('error').textContent='';el('error').dataset.connection='0';}
 }catch(error){
  el('state').textContent='Disconnected';el('error').textContent=error.message;el('error').dataset.connection='1';
  for(const id of ['arm','reset','calibrate','saveGains','applyAxis','saveAxis'])el(id).disabled=true;
 }finally{reading=false;}
}
async function command(path,values={}){
 try{
  const response=await fetch(path,{method:'POST',body:new URLSearchParams(values)});
  const result=await response.json();
  if(!response.ok||!result.ok)throw new Error(result.error||'Command rejected');
  el('error').textContent='';el('error').dataset.connection='0';
 }catch(error){el('error').textContent=error.message;el('error').dataset.connection='0';}
 await status();
}
for(const button of document.querySelectorAll('[data-command]'))
 button.onclick=()=>command('/api/control',{cmd:button.dataset.command});
el('calibrate').onclick=()=>command('/api/calibrate');
for(const [id,path] of [['gains','/api/gains'],['mounting','/api/axis']])
 el(id).onsubmit=event=>{
  event.preventDefault();
  const values=Object.fromEntries(new FormData(event.currentTarget));
  values.save=event.submitter.value;
  command(path,values);
 };
status();setInterval(status,200);
</script>
</body></html>
)HTML";

static String json_string(const char* text) {
    String value(text);
    value.replace("\\", "\\\\");
    value.replace("\"", "\\\"");
    value.replace("\n", "\\n");
    value.replace("\r", "\\r");
    return "\"" + value + "\"";
}

static void result(const char* error = nullptr, int code = 409) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(error ? code : 200, "application/json",
                error ? "{\"ok\":false,\"error\":" + json_string(error) + "}" : "{\"ok\":true}");
}

static void status_response() {
    char fields[384];
    snprintf(fields, sizeof(fields),
        "{\"state\":\"%s\",\"fault\":\"%s\",\"pitch\":%.5f,\"rate\":%.5f,"
        "\"tau\":%.5f,\"i_tau\":%.5f,\"bias\":%.5f,\"kp\":%.5f,\"ki\":%.5f,"
        "\"kd\":%.5f,\"axis\":\"%s\",\"sign\":%d,\"calibration\":\"%s\",\"calibrated\":%s,\"messages\":",
        state_name(), fault_reason, corrected_pitch(), pitch_rate, last_tau, last_i_tau,
        pitch_bias, kp_gain, ki_gain, kd_gain, tilt_axis == TILT_AXIS_X ? "x" : "y",
        tilt_sign, calibration_name(), calibrated ? "true" : "false");
    String messages;
    for (uint8_t i = 0; i < LOG_CAPACITY; ++i) {
        const LogEntry& entry = log_entries[(log_write_index + i) % LOG_CAPACITY];
        if (entry.text[0]) {
            messages += String(entry.ms / 1000.0f, 1) + "s " + entry.text + "\n";
        }
    }
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", String(fields) + json_string(messages.c_str()) + "}");
}

static bool read_gain(const char* name, float maximum, float& value) {
    if (!server.hasArg(name)) return false;
    String text = server.arg(name);
    char* end;
    value = strtof(text.c_str(), &end);
    return end != text.c_str() && *end == '\0' && isfinite(value) && value >= 0 && value <= maximum;
}

static void gains_command() {
    bool save = server.arg("save") == "1";
    if (save && (controller_state != ControllerState::DISARMED || calibration_active())) {
        result("Stop the robot before saving gains");
        return;
    }
    float kp, ki, kd;
    if (!read_gain("kp", 20, kp) || !read_gain("ki", 10, ki) || !read_gain("kd", 10, kd)) {
        result("Expected Kp 0..20, Ki 0..10, Kd 0..10", 400);
        return;
    }
    kp_gain = kp;
    ki_gain = ki;
    kd_gain = kd;
    reset_integral();
    if (save) {
        prefs.putFloat("kp", kp);
        prefs.putFloat("ki", ki);
        prefs.putFloat("kd", kd);
    }
    log_message(save ? "PID gains applied and saved" : "PID gains applied");
    result();
}

static void axis_command() {
    String axis = server.arg("axis"), sign = server.arg("sign");
    if ((axis != "x" && axis != "y") || (sign != "1" && sign != "-1")) {
        result("Expected axis x/y and sign 1/-1", 400);
        return;
    }
    const char* error = set_tilt_axis(axis == "x" ? TILT_AXIS_X : TILT_AXIS_Y, sign == "1" ? 1 : -1);
    if (!error && server.arg("save") == "1") {
        prefs.putUChar("axis", tilt_axis);
        prefs.putChar("sign", tilt_sign);
        log_message("IMU mounting saved");
    }
    result(error);
}

static void control_command() {
    String cmd = server.arg("cmd");
    if (cmd == "arm") result(arm_robot());
    else if (cmd == "stop") { stop_robot(); result(); }
    else if (cmd == "reset") result(reset_fault());
    else result("Expected arm, stop or reset", 400);
}

void web_setup() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", PAGE); });
    server.on("/api/status", HTTP_GET, status_response);
    server.on("/api/control", HTTP_POST, control_command);
    server.on("/api/calibrate", HTTP_POST, []() { result(start_calibration()); });
    server.on("/api/gains", HTTP_POST, gains_command);
    server.on("/api/axis", HTTP_POST, axis_command);
    server.onNotFound([]() { result("Not found", 404); });
    server.begin();
    log_message("Wi-Fi: connect to TM-Robot and open http://192.168.4.1");
}

void web_loop() { server.handleClient(); }
