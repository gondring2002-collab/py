"""
hello gigin – Full GUI 4 Tab (FINAL)
- TAB 1: ESP32-B Stepper + Relay via HiveMQ Cloud (fix Relay)
- TAB 2: Firebase Viewer + Safe Plot
- TAB 3: OTA Upload
- TAB 4: Monitor Sensors A & C + Auto Sequence
- Auto-save per jam 10x tiap menit dengan log
"""

import os, tkinter as tk
from tkinter import ttk, messagebox
import threading, json, time, datetime, queue
import paho.mqtt.client as mqtt

# Optional Firebase
try:
    import firebase_admin
    from firebase_admin import credentials, db
    FIREBASE_AVAILABLE = True
except:
    FIREBASE_AVAILABLE = False

# Optional matplotlib
try:
    import matplotlib.pyplot as plt
except:
    plt = None

# ---------------- CONFIG ----------------
BROKER = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud"
PORT = 8883
USERNAME = "hivemq.webclient.1765435980051"
PASSWORD = "2:E?Q;RJ08&1cijeFbaT"

TOPIC_A_SENSOR = "sensor/airquality/A"
TOPIC_B_SENSOR = "sensor/airquality/B"
TOPIC_C_SENSOR = "sensor/airquality/C"
TOPIC_D_SENSOR = "sensor/airquality/D"
TOPIC_ESP32_STATUS = "home/esp32/status"

TOPIC_STEPPER  = "home/esp32/stepper"
TOPIC_RELAY_B  = "home/esp32/relay"
TOPIC_OTA_A = "esp32/A/ota"
TOPIC_OTA_B = "esp32/B/ota"
TOPIC_OTA_C = "esp32/C/ota"
TOPIC_OTA_D = "esp32/D/ota"

TOPIC_OTA_STATUS_A = "esp32/A/ota/status"
TOPIC_OTA_STATUS_B = "esp32/B/ota/status"
TOPIC_OTA_STATUS_C = "esp32/C/ota/status"
TOPIC_OTA_STATUS_D = "esp32/D/ota/status"

SERVICE_ACCOUNT_FILE = "iot-airquality-9f42d-firebase-adminsdk-fbsvc-ae01f665fd.json"
DATABASE_URL = "https://iot-airquality-9f42d-default-rtdb.asia-southeast1.firebasedatabase.app/"
FIREBASE_PATH_A = "AirQuality/ESP32A"
FIREBASE_PATH_B = "AirQuality/ESP32B"
FIREBASE_PATH_C = "AirQuality/ESP32C"
FIREBASE_PATH_D = "AirQuality/ESP32D"
DEFAULT_OTA_TOKEN = "MY_SECRET_TOKEN"

# ---------------- GLOBAL ----------------
client = None
latest_A = None
latest_B = None
latest_C = None
latest_D = None

stepper_history = []
log_queue = queue.Queue()
esp32_last_status = None

# ---------------- Firebase init ----------------
ref_A = ref_C = None
if FIREBASE_AVAILABLE:
    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_FILE)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        ref_A = db.reference(FIREBASE_PATH_A)
        ref_B = db.reference(FIREBASE_PATH_B)
        ref_C = db.reference(FIREBASE_PATH_C)
        ref_D = db.reference(FIREBASE_PATH_D)
        ref_calibration_time = db.reference("calibration_time")
        print("Firebase connected OK")
    except Exception as e:
        FIREBASE_AVAILABLE = False
        ref_A = ref_C = ref_B = ref_D = None
        print("Firebase init error:", e)

# ---------------- LOG ----------------
def thread_safe_log(msg):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    log_queue.put(f"[{timestamp}] {msg}")

def process_log_queue():
    while not log_queue.empty():
        msg = log_queue.get()
        txt_tab1.configure(state="normal")
        txt_tab1.insert("end", msg + "\n")
        txt_tab1.see("end")
        txt_tab1.configure(state="disabled")
    root.after(100, process_log_queue)

# ---------------- MQTT callbacks ----------------
def mqtt_on_connect(client_obj, userdata, flags, rc, properties=None):
    if rc == 0:
        thread_safe_log("MQTT connected to HiveMQ Cloud")
        client_obj.subscribe(TOPIC_A_SENSOR)
        client_obj.subscribe(TOPIC_B_SENSOR)
        client_obj.subscribe(TOPIC_C_SENSOR)
        client_obj.subscribe(TOPIC_D_SENSOR)
        client_obj.subscribe(TOPIC_OTA_STATUS_A)
        client_obj.subscribe(TOPIC_OTA_STATUS_B)
        client_obj.subscribe(TOPIC_OTA_STATUS_C)
        client_obj.subscribe(TOPIC_OTA_STATUS_D)
        client_obj.subscribe(TOPIC_ESP32_STATUS)   # <<< TAMBAHAN STEPPER
    else:
        thread_safe_log(f"MQTT connect failed rc={rc}")

def mqtt_on_message(client_obj, userdata, msg):
    global latest_A, latest_B,latest_C,latest_D
    topic = msg.topic
    payload = msg.payload.decode(errors="ignore")
    try: data = json.loads(payload)
    except: data = None

    if topic == TOPIC_A_SENSOR and isinstance(data, dict):
        latest_A = data
        root.after(0, lambda d=data: update_sensor_display("A", d))
    elif topic == TOPIC_B_SENSOR and isinstance(data, dict):
        latest_B = data
        root.after(0, lambda d=data: update_sensor_display("B", d))
    elif topic == TOPIC_C_SENSOR and isinstance(data, dict):
        latest_C = data
        root.after(0, lambda d=data: update_sensor_display("C", d))
    elif topic == TOPIC_D_SENSOR and isinstance(data, dict):
        latest_D = data
        root.after(0, lambda d=data: update_sensor_display("D", d))
    elif topic in (TOPIC_OTA_STATUS_A, TOPIC_OTA_STATUS_B,TOPIC_OTA_STATUS_C, TOPIC_OTA_STATUS_D):
        thread_safe_log(f"OTA status {topic}: {payload}")
        root.after(0, lambda: append_txt_tab3(f"OTA status {topic}: {payload}"))

    elif topic == TOPIC_ESP32_STATUS:
        thread_safe_log(f"ESP32 STATUS → {payload}")
# ---------------- MQTT start/stop ----------------
def start_mqtt():
    global client
    if client is not None:
        thread_safe_log("MQTT already running")
        return
    try:
        client = mqtt.Client(client_id="GUI_C", protocol=mqtt.MQTTv5)
        client.username_pw_set(USERNAME, PASSWORD)
        client.tls_set()
        client.on_connect = mqtt_on_connect
        client.on_message = mqtt_on_message
        thread_safe_log("Connecting to HiveMQ Cloud...")
        client.connect(BROKER, PORT, 60)
        client.loop_start()
    except Exception as e:
        thread_safe_log(f"MQTT start error: {e}")
        messagebox.showerror("MQTT", str(e))
        client = None

def stop_mqtt():
    global client
    try:
        if client:
            client.loop_stop()
            client.disconnect()
            client = None
            thread_safe_log("MQTT stopped")
    except Exception as e:
        thread_safe_log(f"Error stopping MQTT: {e}")

# ---------------- SENSOR DISPLAY ----------------
def update_sensor_display(dev, data):
    if dev == "A":
        tempA_var.set(data.get("temp","-"))
        humA_var.set(data.get("hum","-"))
        pm25A_var.set(data.get("pm25","-"))
        co2A_var.set(data.get("co2","-"))
        pressA_var.set(data.get("press","-"))
    elif dev == "B":
        tempB_var.set(data.get("temp","-"))
        humB_var.set(data.get("hum","-"))
        pm25B_var.set(data.get("pm25","-"))
        co2B_var.set(data.get("co2","-"))       
        
    elif dev == "C":
        tempC_var.set(data.get("temp","-"))
        humC_var.set(data.get("hum","-"))
        pm25C_var.set(data.get("pm25","-"))
        co2C_var.set(data.get("co2","-"))    
    else:
        tempD_var.set(data.get("temp","-"))
        humD_var.set(data.get("hum","-"))
        pm25D_var.set(data.get("pm25","-"))
        co2D_var.set(data.get("co2","-"))

# ---------------- OTA ----------------
def append_txt_tab3(msg):
    txt_tab3.configure(state="normal")
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    txt_tab3.insert("end", f"[{ts}] {msg}\n")
    txt_tab3.see("end")
    txt_tab3.configure(state="disabled")

def send_ota_command():
    device = ota_device_var.get()
    url = ota_url_var.get().strip()
    ver = ota_version_var.get().strip()
    token = ota_token_var.get().strip() or DEFAULT_OTA_TOKEN
    if not url:
        messagebox.showwarning("OTA","URL kosong")
        return
    topic = TOPIC_OTA_A if device=="A" else TOPIC_OTA_B
    payload = {"cmd":"update","url":url,"version":ver,"token":token}
    global client
    if client is None:
        thread_safe_log("MQTT not connected - cannot send OTA")
        append_txt_tab3("MQTT not connected - OTA not sent")
        return
    client.publish(topic, json.dumps(payload))
    thread_safe_log(f"OTA sent to {device}: {url}")
    append_txt_tab3(f"OTA sent to {device}: {url}")

# ---------------- Stepper + Relay ----------------
def send_stepper_cmd(cmd):
    global client
    if client is None:
        thread_safe_log("MQTT not connected - cannot send stepper cmd")
        return
    client.publish(TOPIC_STEPPER, cmd)
    thread_safe_log(f"Stepper cmd sent: {cmd}")
    t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    stepper_history.append({"ts": t, "cmd": cmd})
    history_tree.insert("", "end", values=(t, cmd))

def send_relay(state):
    global client
    if client is None:
        thread_safe_log("MQTT not connected - cannot send relay")
        return
    payload = state  # ON/OFF sesuai ESP32
    thread_safe_log(f"Publishing Relay state: '{payload}' to topic: {TOPIC_RELAY_B}")
    result = client.publish(TOPIC_RELAY_B, payload)
    if result.rc == 0:
        thread_safe_log(f"Relay command '{payload}' published successfully")
    else:
        thread_safe_log(f"Failed to publish relay command '{payload}', rc={result.rc}")

# ---------------- Firebase ----------------
def save_now_to_firebase():
    if not FIREBASE_AVAILABLE:
        messagebox.showwarning("Firebase","Not configured")
        return
    try:
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if latest_A: ref_A.child(now).set(latest_A)
        if latest_B: ref_B.child(now).set(latest_B)
        if latest_C: ref_C.child(now).set(latest_C)
        if latest_D: ref_D.child(now).set(latest_D)
        thread_safe_log(f"Saved manually to Firebase ({now})")
    except Exception as e:
        messagebox.showerror("Firebase", str(e))

def load_history_from_firebase(device):
    if not FIREBASE_AVAILABLE:
        messagebox.showwarning("Firebase","Firebase not available")
        return
    ref = ref_A if device=="A" else ref_C
    data = ref.get() or {}
    fb_tree.delete(*fb_tree.get_children())
    for ts in sorted(data.keys()):
        record = data[ts]
        if isinstance(record, dict):
            fb_tree.insert("", "end", values=(ts,
                                              record.get("temp",""),
                                              record.get("hum",""),
                                              record.get("co2",""),
                                              record.get("pm25","")))

# ---------------- Plot ----------------
def plot_graph(device, sensor):
    if not plt or not FIREBASE_AVAILABLE:
        messagebox.showwarning("Plot","matplotlib or Firebase not available")
        return
    ref = ref_A if device=="A" else ref_C
    data = ref.get() or {}
    if not data:
        messagebox.showinfo("Plot","No data")
        return
    ts_list = sorted(data.keys())
    x = []
    for ts in ts_list:
        try: x.append(datetime.datetime.strptime(ts, "%Y-%m-%d_%H-%M-%S"))
        except: x.append(ts)
    plt.figure(figsize=(8,4))
    for s in ["temp","hum","co2","pm25"]:
        if sensor=="all" or sensor==s:
            y = [ (data[ts].get(s,0) if isinstance(data[ts],dict) else 0) for ts in ts_list ]
            plt.plot(x, y, label=s)
    plt.xticks(rotation=45); plt.legend(); plt.tight_layout(); plt.show()

def plot_graph_popup():
    if not FIREBASE_AVAILABLE:
        messagebox.showwarning("Plot","Firebase tidak tersedia")
        return
    popup = tk.Toplevel()
    popup.title("Plot Grafik")
    tk.Label(popup,text="Device:").pack(pady=2)
    device_var = tk.StringVar(value="A")
    ttk.Combobox(popup,textvariable=device_var,values=["A","C"],state="readonly").pack(pady=2)
    tk.Label(popup,text="Sensor:").pack(pady=2)
    sensor_var = tk.StringVar(value="all")
    ttk.Combobox(popup,textvariable=sensor_var,values=["all","temp","hum","co2","pm25"],state="readonly").pack(pady=2)
    ttk.Button(popup,text="Plot",command=lambda: safe_plot(device_var.get(),sensor_var.get())).pack(pady=5)

def safe_plot(device,sensor):
    try: plot_graph(device,sensor)
    except Exception as e:
        messagebox.showerror("Plot Error", f"Gagal plot: {e}")

# ---------------- Auto-save ----------------
def auto_save_loop():
    while True:
        now = datetime.datetime.now()
        next_hour = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)
        thread_safe_log(f"Menunggu auto-save jam berikutnya ({next_hour.strftime('%H:%M:%S')})")
        sleep_sec = (next_hour - now).total_seconds()
        time.sleep(max(1, sleep_sec))
        for i in range(10):
            t = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            if FIREBASE_AVAILABLE:
                try:
                    if latest_A: ref_A.child(t).set(latest_A)
                    if latest_B: ref_B.child(t).set(latest_B)
                    if latest_C: ref_C.child(t).set(latest_C)
                    if latest_D: ref_D.child(t).set(latest_D)
                    thread_safe_log(f"Auto-save data {t}")
                except Exception as e:
                    thread_safe_log(f"Auto-save error: {e}")
            time.sleep(60)

# ---------------- Tab4 Auto Sequence ----------------
def log4(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    log_tab4.insert("end", f"[{ts}] {msg}\n")
    log_tab4.see("end")
"""
def run_sequence():

    def ui(fn):
        root.after(0, fn)

    def thread_func():

        # 1️⃣ Stepper mundur sampai limit
        ui(lambda: log4("Stepper mundur sampai limit"))
        ui(lambda: send_stepper_cmd("backward"))

     
        time.sleep(60)

        # 2️⃣ Pompa vakum ON 60 detik
        ui(lambda: (
            send_relay("ON1"),
            log4("Pompa vakum ON (60 detik)")
        ))

        # 3️⃣ Calibration time 60 detik (per 10 detik)
        save_calibration_time_60s()

        # 4️⃣ Pompa vakum OFF
        ui(lambda: (
            send_relay("OFF1"),
            log4("Pompa vakum OFF")
        ))

        # 5️⃣ Vent ON
        ui(lambda: (
            send_relay("ON2"),
            log4("VENT ON")
        ))

        vent_start = time.time()
        while time.time() - vent_start < 30:
           # if latest_A is not None and latest_A >= PRESSURE_NORMAL:
            #    break
            time.sleep(1)

        ui(lambda: (
            send_relay("OFF2"),
            log4("VENT OFF")
        ))

        # 6️⃣ Stepper maju
        ui(lambda: (
            send_stepper_cmd("forward"),
            log4("Stepper maju")
        ))

        ui(lambda: log4("Sequence selesai"))

    threading.Thread(target=thread_func, daemon=True).start()
"""

def run_sequence():

    def thread_func():

        log4("Stepper mundur sampai limit")
        send_stepper_cmd("backward")

        time.sleep(60)

        # 2️⃣ Pompa vakum ON
        send_relay("ON1")
        log4("Pompa vakum ON (60 detik)")

        # 3️⃣ Calibration time (60 detik, per 10 detik)
        save_calibration_time_60s()

        # 4️⃣ Pompa vakum OFF
        send_relay("OFF1")
        log4("Pompa vakum OFF")

        # 5️⃣ Vent ON
        send_relay("ON2")
        log4("VENT ON")

        vent_start = time.time()
        while time.time() - vent_start < 30:
            time.sleep(1)

        send_relay("OFF2")
        log4("VENT OFF")

        # 6️⃣ Stepper maju
        send_stepper_cmd("forward")
        log4("Stepper maju")

        log4("Sequence selesai")

    threading.Thread(target=thread_func, daemon=True).start()
"""
def run_sequence():

    def thread_func():
        log4("Run sequence: Relay ON1")

        # 🔴 INI KUNCI UTAMA
        root.after(0, btn_relay1_on)

    threading.Thread(target=thread_func, daemon=True).start()
"""
#--------------------------------------------------------------------------------  
def mqtt_on_message(client_obj, userdata, msg):
    global latest_A, latest_B, latest_C, latest_D, esp32_last_status

    topic = msg.topic
    payload = msg.payload.decode(errors="ignore")

    try:
        data = json.loads(payload)
    except:
        data = None

    # ================= SENSOR =================
    if topic == TOPIC_A_SENSOR and isinstance(data, dict):
        latest_A = data
        root.after(0, lambda d=data: update_sensor_display("A", d))

    elif topic == TOPIC_B_SENSOR and isinstance(data, dict):
        latest_B = data
        root.after(0, lambda d=data: update_sensor_display("B", d))

    elif topic == TOPIC_C_SENSOR and isinstance(data, dict):
        latest_C = data
        root.after(0, lambda d=data: update_sensor_display("C", d))

    elif topic == TOPIC_D_SENSOR and isinstance(data, dict):
        latest_D = data
        root.after(0, lambda d=data: update_sensor_display("D", d))

    # ================= OTA STATUS =================
    elif topic in (
        TOPIC_OTA_STATUS_A,
        TOPIC_OTA_STATUS_B,
        TOPIC_OTA_STATUS_C,
        TOPIC_OTA_STATUS_D
    ):
        thread_safe_log(f"OTA status {topic}: {payload}")
        root.after(0, lambda:
            append_txt_tab3(f"OTA status {topic}: {payload}")
        )

    # ================= ESP32 STATUS =================
    elif topic == TOPIC_ESP32_STATUS:
        esp32_last_status = payload
        thread_safe_log(f"ESP32 STATUS → {payload}")
        root.after(0, lambda:
            log4(f"ESP32 STATUS → {payload}")
        )

#------------------------------------------------------------------------------------        
def btn_relay1_on():
    log4("Kirim relay ON1")
    send_relay("ON2")
#-----------------------------------------------------------------------------------------
def save_calibration_time_60s():
    if not FIREBASE_AVAILABLE:
        return

    for i in range(6):  # 0,10,20,30,40,50 detik
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        data = {
            "timestamp": ts,
            "mode": "calibration_time",
            "ESP32_A": latest_A,
            "ESP32_B": latest_B,
            "ESP32_C": latest_C,
            "ESP32_D": latest_D
        }

        ref_calibration_time.child(ts).set(data)

        root.after(0, lambda d=data:
            log4(
                f"CAL-TIME [{i*10}s] "
                f"A:{d['ESP32_A']} "
                f"B:{d['ESP32_B']} "
                f"C:{d['ESP32_C']} "
                f"D:{d['ESP32_D']}"
            )
        )

        time.sleep(10)

    
# ---------------- Fungsi kirim mode ----------------
#def send_mode(value):
#    global client
#    if client is None:
#        thread_safe_log("MQTT tidak connected - mode tidak terkirim")
#        return
#    topic = "home/esp32/mode"  # Topik MQTT untuk mode
#    client.publish(topic, value)
#    thread_safe_log(f"Mode dikirim: {value}")
    
    
def send_mode(value):
    global client

    if client is None:
        thread_safe_log("MQTT tidak connected - mode tidak terkirim")
        return

    # Validasi sederhana: harus format x%_y%
    if "_" not in value:
        thread_safe_log(f"Format mode salah: {value}")
        return

    try:
        left, right = value.split("_")
        if not left.endswith("%") or not right.endswith("%"):
            raise ValueError
        int(left.replace("%",""))
        int(right.replace("%",""))
    except:
        thread_safe_log(f"Mode tidak valid: {value}")
        return

    topic = "home/esp32/mode"
    client.publish(topic, value)

    thread_safe_log(f"Mode dikirim: {value}")    

# ---------------- GUI ----------------
root = tk.Tk()
root.title("ESP32 Manager – Full GUI")
root.geometry("1100x700")

notebook = ttk.Notebook(root)
tab1 = ttk.Frame(notebook)
tab2 = ttk.Frame(notebook)
tab3 = ttk.Frame(notebook)
tab4 = ttk.Frame(notebook)
notebook.add(tab1, text="MQTT (A/C + B Stepper)")
notebook.add(tab2, text="Firebase History")
notebook.add(tab3, text="OTA")
notebook.add(tab4, text="Auto Sequence")
notebook.pack(fill="both", expand=True)

# ---------------- Tab1 ----------------
left1 = ttk.Frame(tab1); left1.pack(side="left", fill="y", padx=10,pady=8)
left2 = ttk.Frame(tab1); left2.pack(side="left", fill="y", padx=10,pady=8)
left3 = ttk.Frame(tab1); left3.pack(side="left", fill="y", padx=10,pady=8)

#relay_frame = ttk.Frame(main_tab1)
#relay_frame.pack(side="left", fill="y", padx=5, pady=5)

ttk.Label(left1,text="MQTT").pack(anchor="w")
ttk.Button(left1,text="Connect MQTT",command=lambda: threading.Thread(target=start_mqtt,daemon=True).start()).pack(fill="x", pady=5)
ttk.Button(left1,text="Stop MQTT",command=lambda: threading.Thread(target=stop_mqtt,daemon=True).start()).pack(fill="x", pady=5)




# ESP32-A
gA = ttk.LabelFrame(left1,text="ESP32-A"); gA.pack(fill="x", pady=5)
tempA_var=tk.StringVar(value="-"); humA_var=tk.StringVar(value="-")
pm25A_var=tk.StringVar(value="-"); co2A_var=tk.StringVar(value="-")
pressA_var=tk.StringVar(value="-");
ttk.Label(gA,text="Temp:").grid(row=0,column=0); ttk.Label(gA,textvariable=tempA_var).grid(row=0,column=1)
ttk.Label(gA,text="Hum:").grid(row=1,column=0); ttk.Label(gA,textvariable=humA_var).grid(row=1,column=1)
ttk.Label(gA,text="PM2.5:").grid(row=2,column=0); ttk.Label(gA,textvariable=pm25A_var).grid(row=2,column=1)
ttk.Label(gA,text="CO2:").grid(row=3,column=0); ttk.Label(gA,textvariable=co2A_var).grid(row=3,column=1)
ttk.Label(gA,text="PamB:").grid(row=4,column=0); ttk.Label(gA,textvariable=pressA_var).grid(row=4,column=1)
# ESP32-B
gB = ttk.LabelFrame(left1,text="ESP32-B"); gB.pack(fill="x", pady=5)
tempB_var=tk.StringVar(value="-"); humB_var=tk.StringVar(value="-")
pm25B_var=tk.StringVar(value="-"); co2B_var=tk.StringVar(value="-")
ttk.Label(gB,text="Temp:").grid(row=0,column=0); ttk.Label(gB,textvariable=tempB_var).grid(row=0,column=1)
ttk.Label(gB,text="Hum:").grid(row=1,column=0); ttk.Label(gB,textvariable=humB_var).grid(row=1,column=1)
ttk.Label(gB,text="PM2.5:").grid(row=2,column=0); ttk.Label(gB,textvariable=pm25B_var).grid(row=2,column=1)
ttk.Label(gB,text="CO2:").grid(row=3,column=0); ttk.Label(gB,textvariable=co2B_var).grid(row=3,column=1)

# ESP32-C
gC = ttk.LabelFrame(left1,text="ESP32-C"); gC.pack(fill="x", pady=5)
tempC_var=tk.StringVar(value="-"); humC_var=tk.StringVar(value="-")
pm25C_var=tk.StringVar(value="-"); co2C_var=tk.StringVar(value="-")
ttk.Label(gC,text="Temp:").grid(row=0,column=0); ttk.Label(gC,textvariable=tempC_var).grid(row=0,column=1)
ttk.Label(gC,text="Hum:").grid(row=1,column=0); ttk.Label(gC,textvariable=humC_var).grid(row=1,column=1)
ttk.Label(gC,text="PM2.5:").grid(row=2,column=0); ttk.Label(gC,textvariable=pm25C_var).grid(row=2,column=1)
ttk.Label(gC,text="CO2:").grid(row=3,column=0); ttk.Label(gC,textvariable=co2C_var).grid(row=3,column=1)

# ESP32-D
gD = ttk.LabelFrame(left1,text="ESP32-D"); gD.pack(fill="x", pady=5)
tempD_var=tk.StringVar(value="-"); humD_var=tk.StringVar(value="-")
pm25D_var=tk.StringVar(value="-"); co2D_var=tk.StringVar(value="-")
ttk.Label(gD,text="Temp:").grid(row=0,column=0); ttk.Label(gD,textvariable=tempD_var).grid(row=0,column=1)
ttk.Label(gD,text="Hum:").grid(row=1,column=0); ttk.Label(gD,textvariable=humD_var).grid(row=1,column=1)
ttk.Label(gD,text="PM2.5:").grid(row=2,column=0); ttk.Label(gD,textvariable=pm25D_var).grid(row=2,column=1)
ttk.Label(gD,text="CO2:").grid(row=3,column=0); ttk.Label(gD,textvariable=co2D_var).grid(row=3,column=1)

# Stepper + Relay
#gB = ttk.LabelFra3me(left1,text="ESP32-B Stepper & Relay"); gB.pack(fill="x", pady=5)
#ttk.Button(gB,text="FORWARD",command=lambda: send_stepper_cmd("forward")).pack(fill="x", pady=2)
#ttk.Button(gB,text="BACKWARD",command=lambda: send_stepper_cmd("backward")).pack(fill="x", pady=2)
#ttk.Button(gB,text="STOP",command=lambda: send_stepper_cmd("stop")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay ON 1",command=lambda: send_relay("ON")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay OFF 1",command=lambda: send_relay("OFF")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay ON 2",command=lambda: send_relay("ON")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay OFF 2",command=lambda: send_relay("OFF")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay ON 3",command=lambda: send_relay("ON")).pack(fill="x", pady=2)
#ttk.Button(gB,text="Relay OFF 3",command=lambda: send_relay("OFF")).pack(fill="x", pady=2)

#history_tree = ttk.Treeview(gB, columns=("ts","cmd"), show="headings")
#history_tree.heading("ts", text="Timestamp"); history_tree.heading("cmd", text="Command")
#history_tree.pack(fill="both", expand=True, pady=5)

ctrl = ttk.LabelFrame(left2, text="Control ESP32-B Stepper & Relay")
ctrl.pack(fill="x")

#ttk.Button(ctrl, text="Relay 1 ON").pack(fill="x")
#gB = ttk.LabelFrame(left1,text="ESP32-B Stepper & Relay"); gB.pack(fill="x", pady=5)
ttk.Button(ctrl,text="FORWARD",command=lambda: send_stepper_cmd("forward")).pack(fill="x", pady=2)
ttk.Button(ctrl,text="BACKWARD",command=lambda: send_stepper_cmd("backward")).pack(fill="x", pady=2)
ttk.Button(ctrl,text="STOP",command=lambda: send_stepper_cmd("Stop")).pack(fill="x", pady=2)
relay1 = ttk.LabelFrame(ctrl,text="Relay 1 :POMPA VAKUM"); relay1.pack(fill="x", pady=5)
ttk.Button(relay1,text="Relay ON 1",command=lambda: send_relay("ON1")).pack(fill="x", pady=2)
ttk.Button(relay1,text="Relay OFF 1",command=lambda: send_relay("OFF1")).pack(fill="x", pady=2)
relay2 = ttk.LabelFrame(ctrl,text="Relay 2"); relay2.pack(fill="x", pady=5)
ttk.Button(relay2,text="Relay ON 2",command=lambda: send_relay("ON2")).pack(fill="x", pady=2)
ttk.Button(relay2,text="Relay OFF 2",command=lambda: send_relay("OFF2")).pack(fill="x", pady=2)
relay3 = ttk.LabelFrame(ctrl,text="Relay 3"); relay3.pack(fill="x", pady=5)
ttk.Button(relay3,text="Relay ON 3",command=lambda: send_relay("ON3")).pack(fill="x", pady=2)
ttk.Button(relay3,text="Relay OFF 3",command=lambda: send_relay("OFF3")).pack(fill="x", pady=2)
history_tree = ttk.Treeview(ctrl, columns=("ts","cmd"), show="headings", height=6)
history_tree.heading("ts", text="Timestamp"); history_tree.heading("cmd", text="Command")
history_tree.pack(fill="both", expand=True, pady=4)

log = ttk.LabelFrame(left3, text="Control ESP32-B Stepper & Relay")
log.pack(fill="x")


txt_tab1 = tk.Text(log,height=35,state="disabled"); txt_tab1.pack(fill="both",expand=True,padx=10,pady=10)

combo_var = tk.StringVar()
combo_var1 = tk.StringVar()
ttk.Label(ctrl, text="Pilih Mode:").pack(pady=(5,2))
combo = ttk.Combobox(ctrl, textvariable=combo_var, values=["0%","20%","40%","60%","80%","100%"], state="readonly")
combo.current(0)
combo.pack(side="left",padx=4,pady=2)
combo1 = ttk.Combobox(ctrl, textvariable=combo_var1, values=["0%","20%","40%","60%","80%","100%"], state="readonly")
combo1.current(0)
combo1.pack(side="left",padx=4,pady=2)
#ttk.Button(ctrl, text="Kirim Mode", command=lambda: send_mode(combo_var.get()+"_"+combo_var1.get()).pack(fill="x", pady=2)
ttk.Button(ctrl, text="Kirim Mode", command=lambda: send_mode(combo_var.get() + "_" + combo_var1.get())).pack(fill="x", pady=2)

# ---------------- Tab2 ----------------
top2 = ttk.Frame(tab2); top2.pack(fill="x", padx=10,pady=5)
ttk.Button(top2,text="Load A",command=lambda: load_history_from_firebase("A")).pack(side="left",padx=4)
ttk.Button(top2,text="Load B",command=lambda: load_history_from_firebase("B")).pack(side="left",padx=4)
ttk.Button(top2,text="Load C",command=lambda: load_history_from_firebase("C")).pack(side="left",padx=4)
ttk.Button(top2,text="Load D",command=lambda: load_history_from_firebase("D")).pack(side="left",padx=4)
ttk.Button(top2,text="Save Manual",command=save_now_to_firebase).pack(side="left",padx=10)
ttk.Button(top2,text="Plot Grafik",command=lambda: plot_graph_popup()).pack(side="left",padx=4)
fb_cols = ("timestamp","temp","hum","co2","pm25")
fb_tree = ttk.Treeview(tab2,columns=fb_cols,show="headings")
for c in fb_cols: fb_tree.heading(c,text=c.upper())
fb_tree.pack(fill="both",expand=True,padx=10,pady=10)

# ---------------- Tab3 ----------------
f3 = ttk.LabelFrame(tab3,text="OTA Control"); f3.pack(fill="x", padx=10,pady=10)
ota_device_var = tk.StringVar(value="A")
ttk.Label(f3,text="Device:").grid(row=0,column=0)
ttk.Combobox(f3,textvariable=ota_device_var,values=["A","B"],width=10,state="readonly").grid(row=0,column=1)
ota_url_var = tk.StringVar()
ttk.Label(f3,text="Firmware URL:").grid(row=1,column=0,columnspan=2)
ttk.Entry(f3,textvariable=ota_url_var,width=60).grid(row=2,column=0,columnspan=2,pady=3)
ota_version_var = tk.StringVar()
ttk.Label(f3,text="Version:").grid(row=3,column=0)
ttk.Entry(f3,textvariable=ota_version_var).grid(row=3,column=1)
ota_token_var = tk.StringVar(value=DEFAULT_OTA_TOKEN)
ttk.Label(f3,text="Token:").grid(row=4,column=0)
ttk.Entry(f3,textvariable=ota_token_var).grid(row=4,column=1)
ttk.Button(f3,text="SEND OTA",command=send_ota_command).grid(row=5,column=0,pady=6)
lf3 = ttk.LabelFrame(tab3,text="OTA / Log Window"); lf3.pack(fill="both",expand=True,padx=10,pady=10)
txt_tab3 = tk.Text(lf3,height=12,state="disabled"); txt_tab3.pack(fill="both",expand=True)

# ---------------- Tab4 ----------------
ttk.Button(tab4,text="Jalankan",command=run_sequence,width=12).pack(pady=5)
log_tab4 = tk.Text(tab4,height=15); log_tab4.pack(fill="both",expand=True,padx=10,pady=10)

# ---------------- Graceful shutdown ----------------
def on_close():
    if messagebox.askokcancel("Quit", "Tutup aplikasi?"):
        stop_mqtt()
        root.destroy()
root.protocol("WM_DELETE_WINDOW", on_close)

# ---------------- START ----------------
threading.Thread(target=start_mqtt,daemon=True).start()
threading.Thread(target=auto_save_loop,daemon=True).start()
root.after(100, process_log_queue)
root.mainloop()
