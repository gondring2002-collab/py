"""
hello gigin – FULL GUI (FINAL – FIXED)
ESP32-A : Air Quality Sensor
ESP32-B : Stepper + Relay
"""

import tkinter as tk
from tkinter import ttk
import threading, json, time, datetime, queue
import paho.mqtt.client as mqtt

# ================= MQTT CONFIG =================
BROKER = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud"
PORT = 8883
USERNAME = "hivemq.webclient.1765435980051"
PASSWORD = "2:E?Q;RJ08&1cijeFbaT"


SERVICE_ACCOUNT_FILE = "iot-airquality-9f42d-firebase-adminsdk-fbsvc-ae01f665fd.json"
DATABASE_URL = "https://iot-airquality-9f42d-default-rtdb.asia-southeast1.firebasedatabase.app/"
FIREBASE_PATH_A = "AirQuality/ESP32A"
FIREBASE_PATH_B = "AirQuality/ESP32B"
FIREBASE_PATH_C = "AirQuality/ESP32C"
FIREBASE_PATH_D = "AirQuality/ESP32D"
DEFAULT_OTA_TOKEN = "MY_SECRET_TOKEN"

# ===== ESP32-A (SENSOR) =====
TOPIC_A_SENSOR = "sensor/airquality/A"
TOPIC_B_SENSOR = "sensor/airquality/B"
TOPIC_C_SENSOR = "sensor/airquality/C"
TOPIC_D_SENSOR = "sensor/airquality/D"

TOPIC_OTA_A = "esp32/A/ota"
TOPIC_OTA_STATUS_A = "esp32/A/ota/status"

# ===== ESP32-B (MEKANIK) =====
TOPIC_STEP1 = "home/esp32/stepper/1"
TOPIC_STEP2 = "home/esp32/stepper/2"
TOPIC_RELAY = "home/esp32/relay"
TOPIC_STATUS = "home/esp32/status"

# ================= GLOBAL =================
client = None
log_queue = queue.Queue()
auto_running = False
latest_status = {
    's1_state': 0,
    's1_pos': 0,
    's1_lim': 1,
    's2_state': 0,
    's2_pos': 0,
    's2_lim': 1,
    'r1': 0,
    'r2': 0,
    'r3': 0
}

latest_A = {}
latest_B = {}
latest_C = {}
latest_D = {}

try:
    import firebase_admin
    from firebase_admin import credentials, db
    FIREBASE_AVAILABLE = True
except Exception as e:
    FIREBASE_AVAILABLE = False
    print("Firebase library missing:", e)
    
CAL_DURATION_SEC = 960 #300   # 5 menit
CAL_INTERVAL_SEC = 10   # tiap 10 detik
calibration_running = False

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


import queue
popup_queue = queue.Queue()

# ================= LOG =================
def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    log_queue.put(f"[{ts}] {msg}")

def process_log():
    while not log_queue.empty():
        txt_log.configure(state="normal")
        txt_log.insert("end", log_queue.get() + "\n")
        txt_log.see("end")
        txt_log.configure(state="disabled")
    root.after(100, process_log)

# ================= MQTT =================
def mqtt_on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        log("MQTT CONNECTED")
        client.subscribe([
            (TOPIC_A_SENSOR, 0),
            (TOPIC_OTA_STATUS_A, 0),
            (TOPIC_STATUS, 0),
        ])
    else:
        log(f"MQTT ERROR rc={rc}")

def mqtt_on_message(client, userdata, msg):
    global latest_A, latest_B, latest_C, latest_D, latest_status

    payload = msg.payload.decode(errors="ignore")

    if msg.topic == TOPIC_A_SENSOR:
        try:
            data = json.loads(payload)
            latest_A = data
            print("SAVE A:", latest_A)

            tempA_var.set(data.get("temp","-"))
            humA_var.set(data.get("hum","-"))
            pm25A_var.set(data.get("pm25","-"))
            co2A_var.set(data.get("co2","-"))
            pressA_var.set(data.get("press","-"))

        except Exception as e:
            log(f"ESP32-A payload error: {e}")

    elif msg.topic == TOPIC_OTA_STATUS_A:
        log(f"OTA A → {payload}")

    elif msg.topic == TOPIC_STATUS:
        log(f"ESP32-B STATUS → {payload}")
        log_tab4.insert("end", payload + "\n")
        log_tab4.see("end")

        try:
            parts = payload.split(',')
            for p in parts:
                k, v = p.split('=')
                latest_status[k.strip()] = int(v.strip())
        except:
            log("Failed to parse ESP32-B status")

def get_latest_status():
    #return latest_status.copy()]'[=-[]'/'
    return latest_status.copy()

def start_mqtt():
    global client
    if client:
        return
    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.username_pw_set(USERNAME, PASSWORD)
    client.tls_set()
    client.on_connect = mqtt_on_connect
    client.on_message = mqtt_on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()
    log("Connecting MQTT...")

def stop_mqtt():
    global client
    if client:
        client.loop_stop()
        client.disconnect()
        client = None
        log("MQTT stopped")

# ================= CONTROL ESP32-B =================
def send_stepper(topic, cmd):
    if not client or auto_running:
        return
    client.publish(topic, cmd)
    log(f"Stepper → {topic} : {cmd}")

def send_relay(cmd):
    if not client:
        return
    client.publish(TOPIC_RELAY, cmd)
    log(f"Relay → {cmd}")
    
def send_stepper_auto(topic, direction, timeout=30):
    # Kirim perintah stepper
    client.publish(topic, direction)
    start = time.time()
    while time.time() - start < timeout:
        # baca status dari ESP32 (MQTT topicStatus)
        status = get_latest_status()  # dictionary dari topicStatus terakhir
        if topic == TOPIC_STEP1 and status['s1_lim'] == 0:  # LOW = limit switch ditekan
            log_tab4.insert("end", "Stepper1 reached limit switch!\n")
            client.publish(topic, "stop")
            break
        if topic == TOPIC_STEP2 and status['s2_lim'] == 0:
            log_tab4.insert("end", "Stepper2 reached limit switch!\n")
            client.publish(topic, "stop")
            break
        time.sleep(0.1)

def save_calibration_time_60s():
    if not FIREBASE_AVAILABLE:
        return
    print("SAVE A:", latest_A)   # ✅ DI SINI
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
            log_tab4.insert(
                "end",
                f"CAL-TIME [{i*10}s] "
                f"A:{d['ESP32_A']} "
                f"B:{d['ESP32_B']} "
                f"C:{d['ESP32_C']} "
                f"D:{d['ESP32_D']}\n"
            ),
            #log_tab4.see("end") 
        )
        root.after(0, lambda d=data:
    log_tab4.insert(
        "end",
        f"CAL-TIME [{i*10}s] "
        f"A:{d['ESP32_A']} "
        f"B:{d['ESP32_B']} "
        f"C:{d['ESP32_C']} "
        f"D:{d['ESP32_D']}\n"
    ),
    #log_tab4.see("end") 
)

        time.sleep(10)  
        
def send_relay_auto(cmd, timeout=5):
    """
    Kirim perintah relay dan tunggu status MQTT sesuai
    Contoh: send_relay_auto("OFF2")
    """

    if not client:
        return

    # mapping relay & target state
    relay_map = {
        "ON1":  ("r1", 1),
        "OFF1": ("r1", 0),
        "ON2":  ("r2", 1),
        "OFF2": ("r2", 0),
        "ON3":  ("r3", 1),
        "OFF3": ("r3", 0),
    }

    if cmd not in relay_map:
        log_tab4.insert("end", f"Relay AUTO INVALID CMD: {cmd}\n")
        return

    relay_key, target_state = relay_map[cmd]

    # publish command
    client.publish(TOPIC_RELAY, cmd)
    log_tab4.insert("end", f"Relay AUTO → {cmd}\n")

    start_time = time.time()

    while auto_running:
        current = latest_status.get(relay_key)

        # status sudah sesuai
        if current == target_state:
            log_tab4.insert(
                "end",
                f"{relay_key.upper()} OK ({current})\n"
            )
            break

        # timeout safety
        if time.time() - start_time > timeout:
            log_tab4.insert(
                "end",
                f"{relay_key.upper()} TIMEOUT (status={current})\n"
            )
            break

        time.sleep(0.05)

def wait_60s_with_counter():
    for i in range(1, 61):
        root.after(0, lambda s=i: (
            log_tab4.insert("end", f"Menunggu... {s}/60 detik\n"),
            log_tab4.see("end")
        ))
        time.sleep(1)
# ================= AUTO SEQUENCE (ESP32-B) =================
def run_sequence():
    global auto_running

    def worker():
        global auto_running
        auto_running = True
        log_tab4.insert("end", "=== AUTO START ===\n")
        log_tab4.insert("end", "STEPPER HOMING START\n")
        #send_stepper(TOPIC_STEP2, "backward")
        #send_stepper(TOPIC_STEP1, "backward")
        #send_stepper_auto(TOPIC_STEP2, "backward")
        #send_stepper_auto(TOPIC_STEP1, "backward")
        #Time.sleep(5)
        log_tab4.insert("end", "STEPPER HOMING FINISH\n")
        time.sleep(1)
        log_tab4.insert("end", "VAKUUM ON START\n")
        send_relay("ON2")
        send_relay("ON3")
        send_relay("OFF1")
        log_tab4.insert("end", "VAKUUM ON FINISH\n")
        wait_60s_with_counter()
        send_relay("ON2")
        send_relay("ON3")
        send_relay("ON1")
        #send_relay_auto("OFF2")   # Vacuum ON
        #send_relay_auto("OFF3")   # Vacuum ON
        #send_relay_auto("OFF1")   # Vacuum ON
        #log_tab4.insert("end", "Vacuum ON\n")
        #log_tab4.insert("end", "VAKUUM ON\n")
        time.sleep(1)
        
        log_tab4.insert("end", "CALIBRATION START\n")
        save_calibration_time_60s()
        log_tab4.insert("end", "CALIBRATION FINISH\n")  
        time.sleep(1)
        log_tab4.insert("end", "VAKUUM VENT START\n")
        send_relay("ON1")
        send_relay("OFF2")
        send_relay("OFF3")
        wait_60s_with_counter()
        time.sleep(1) 
        log_tab4.insert("end", "VAKUUM VENT FINISH\n")
        #log_tab4.insert("end", "Calibration\n")
        #send_relay("ON1")
        #send_stepper(TOPIC_STEP2, "forward")
        #send_stepper(TOPIC_STEP1, "forward")
        #time.sleep(60)
        #send_stepper(TOPIC_STEP2, "stop")
        #send_stepper(TOPIC_STEP1, "stop")

        log_tab4.insert("end", "=== AUTO END ===\n")
        auto_running = False

    threading.Thread(target=worker, daemon=True).start()
    
def start_calibration():
    global calibration_running
    if calibration_running:
        return

    calibration_running = True
    threading.Thread(
        target=save_calibration_continuous,
        daemon=True
    ).start()
    
    
    
    
def stop_calibration():
    global calibration_running
    calibration_running = False
    
    
    
def save_calibration_continuous():
 
    global calibration_running

    if not FIREBASE_AVAILABLE:
        calibration_running = False
        return

    time.sleep(2)  # sama kayak versi 60 detik

    while calibration_running:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        ref_calibration_time.child(ts).set({
            "timestamp": ts,
            "mode": "calibration",
            "ESP32_A": latest_A,
            "ESP32_B": latest_B,
            "ESP32_C": latest_C,
            "ESP32_D": latest_D
        })

        root.after(
            0,
            lambda:
                log_tab4.insert(
                    "end",
                    f"CAL "
                    f"A:{latest_A} "
                    f"B:{latest_B} "
                    f"C:{latest_C} "
                    f"D:{latest_D}\n"
                )
        )

        time.sleep(10)


    
# ================= AUTO SEQUENCE 2(ESP32-B) =================
def run_sequence2():
    global auto_running

    def worker():
        global auto_running
        auto_running = True
        log_tab4.insert("end", "=== AUTO START ===\n")
        log_tab4.insert("end", "STEPPER HOMING START\n")
        start_calibration()
        wait_60s_with_counter()
        #send_stepper(TOPIC_STEP2, "backward")
        #send_stepper(TOPIC_STEP1, "backward")
        #send_stepper_auto(TOPIC_STEP2, "backward")
        #send_stepper_auto(TOPIC_STEP1, "backward")
        #Time.sleep(5)
        log_tab4.insert("end", "STEPPER HOMING FINISH\n")
        time.sleep(1)
        log_tab4.insert("end", "VAKUUM  START\n")
        send_relay("ON2")
        send_relay("ON3") #V1 CLOSE
        send_relay("OFF1")   # POMPA ON
        wait_60s_with_counter()
        wait_60s_with_counter()
        time.sleep(5)
        send_relay("ON2")   
        send_relay("ON3")  #V1 CLOSE
        send_relay("ON1")   #POMPA OFF
        log_tab4.insert("end", "VAKUUM FINISH\n")
        time.sleep(5)
        send_relay("ON2")
        send_relay("OFF3")   #V1 OPEN
        send_relay("ON1")    #POMPA OFF
        log_tab4.insert("end", "V1 OPEN\n")
        
           
        log_tab4.insert("end", "CALIBRATION START\n")
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        #save_calibration_time_60s()
        log_tab4.insert("end", "CALIBRATION FINISH\n")  
        
        time.sleep(5)
        log_tab4.insert("end", "VAKUUM VENT START\n")
        send_relay("ON1")
        send_relay("OFF2")
        send_relay("OFF3")
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        time.sleep(1) 
        log_tab4.insert("end", "VAKUUM VENT FINISH\n")
        #log_tab4.insert("end", "Calibration\n")
        #send_relay("ON1")
        #send_stepper(TOPIC_STEP2, "forward")
        #send_stepper(TOPIC_STEP1, "forward")
        #time.sleep(60)
        #send_stepper(TOPIC_STEP2, "stop")
        #send_stepper(TOPIC_STEP1, "stop")
       
        wait_60s_with_counter()
        stop_calibration()
        log_tab4.insert("end", "=== AUTO END ===\n")
        auto_running = False

    threading.Thread(target=worker, daemon=True).start()
    
    
def popup_ok_no(title, msg):
    popup = tk.Toplevel(root)
    popup.title(title)
    popup.geometry("300x120")
    popup.transient(root)

    tk.Label(popup, text=msg).pack(pady=15)

    def ok():
        popup_queue.put(True)
        popup.destroy()

    def no():
        popup_queue.put(False)
        popup.destroy()

    tk.Button(popup, text="OK", width=10, command=ok).pack(side="left", padx=20)
    tk.Button(popup, text="NO", width=10, command=no).pack(side="right", padx=20)

def ask_ok_cancel(title, msg):
    root.after(0, lambda: popup_ok_no(title, msg))
    return popup_queue.get()   # THREAD nunggu jawaban
        
# ================= AUTO SEQUENCE 2(ESP32-B) =================
def run_sequence3():
    global auto_running

    def worker():
        global auto_running
        auto_running = True
        log_tab4.insert("end", "=== AUTO START ===\n")
     

        log_tab4.insert("end", "STEPPER HOMING START\n")
        start_calibration()
      
        #send_stepper(TOPIC_STEP2, "backward")
        #send_stepper(TOPIC_STEP1, "backward")
        send_stepper_auto(TOPIC_STEP2, "backward")
        #send_stepper_auto(TOPIC_STEP1, "backward")
        #Time.sleep(5)
        log_tab4.insert("end", "STEPPER HOMING FINISH\n")
        #wait_60s_with_counter()
        wait_60s_with_counter()
        log_tab4.insert("end", "STEPPER HOMING FINISH\n")
        time.sleep(1)
        log_tab4.insert("end", "VAKUUM  START\n")
        send_relay("ON2")
        send_relay("ON3") #V1 CLOSE
        send_relay("OFF1")   # POMPA ON
        wait_60s_with_counter()
        #wait_60s_with_counter()
        time.sleep(5)
        send_relay("ON2")   
        send_relay("ON3")  #V1 CLOSE
        send_relay("ON1")   #POMPA OFF
        log_tab4.insert("end", "VAKUUM FINISH\n")
        time.sleep(5)
        
        res = ask_ok_cancel("Masukan CO2 1X ", "Lanjutkan?")
        if not res:
            return
        
        
        send_relay("ON2")
        send_relay("OFF3")   #V1 OPEN
        send_relay("ON1")    #POMPA OFF
        log_tab4.insert("end", "V1 OPEN\n")
        
           
        log_tab4.insert("end", "CALIBRATION START\n")
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        #save_calibration_time_60s()
        log_tab4.insert("end", "CALIBRATION FINISH\n")  
        
        time.sleep(5)
        log_tab4.insert("end", "VAKUUM VENT START\n")
        #res = messagebox.askokcancel("Konfirmasi", "Lanjutkan?")
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("ON3")  # V1 off
         
         
        res = ask_ok_cancel("Masukan CO2 2X ", "Lanjutkan?")
        if not res:
            return 
            
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("OFF3")  # V1 OPEN    
        
        wait_60s_with_counter() 
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("ON3")  # V1 off
        
        wait_60s_with_counter()# tunggu
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        time.sleep(1) 
        
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("ON3")  # V1 close
        res = ask_ok_cancel("Masukan Nitrogen", "Lanjutkan?")
        if not res:
            return 
            
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("OFF3")  # V1 OPEN    
        
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        time.sleep(1) 
        log_tab4.insert("end", "VAKUUM VENT FINISH\n")
        send_relay("ON1")  #pompaoff
        send_relay("ON2")  #vent off
        send_relay("ON3")  # V1 OPEN
        #log_tab4.insert("end", "Calibration\n")
        #send_relay("ON1")
        #send_stepper(TOPIC_STEP2, "forward")
        #send_stepper(TOPIC_STEP1, "forward")
        #time.sleep(60)
        #send_stepper(TOPIC_STEP2, "stop")
        #send_stepper(TOPIC_STEP1, "stop")
       
        wait_60s_with_counter()
        stop_calibration()
        log_tab4.insert("end", "=== AUTO END ===\n")
        auto_running = False

    threading.Thread(target=worker, daemon=True).start()
# ================= AUTO SEQUENCE 2(ESP32-B) =================
def run_sequence4():
    global auto_running

    def worker():
        global auto_running
        auto_running = True
        log_tab4.insert("end", "=== AUTO START ===\n")
        start_calibration()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        wait_60s_with_counter()
        stop_calibration()
        #log_tab4.insert("end", "=== AUTO START ===\n")
        #log_tab4.insert("end", "STEPPER HOMING START\n")
        #send_stepper(TOPIC_STEP2, "backward")
        #send_stepper(TOPIC_STEP1, "backward")
        #send_stepper_auto(TOPIC_STEP2, "backward")
        #send_stepper_auto(TOPIC_STEP1, "backward")
        #Time.sleep(5)
        #log_tab4.insert("end", "STEPPER HOMING FINISH\n")
        #time.sleep(1)
        #log_tab4.insert("end", "VAKUUM ON START\n")
        #send_relay("ON2")
        #send_relay("OFF3")
        #send_relay("OFF1")
        #log_tab4.insert("end", "VAKUUM ON FINISH\n")
        #wait_60s_with_counter()
        #send_relay("ON2")
        #send_relay("ON3")
        #send_relay("ON1")
        #send_relay_auto("OFF2")   # Vacuum ON
        #send_relay_auto("OFF3")   # Vacuum ON
        #send_relay_auto("OFF1")   # Vacuum ON
        #log_tab4.insert("end", "Vacuum ON\n")
        #log_tab4.insert("end", "VAKUUM ON\n")
        #time.sleep(1)
        
        #log_tab4.insert("end", "CALIBRATION START\n")
        #save_calibration_time_60s()
        #log_tab4.insert("end", "CALIBRATION FINISH\n")  
        #time.sleep(1)
        #log_tab4.insert("end", "VAKUUM VENT START\n")
        #send_relay("ON1")
        #send_relay("OFF2")
        #send_relay("OFF3")
        #wait_60s_with_counter()
        #time.sleep(1) 
        #log_tab4.insert("end", "VAKUUM VENT FINISH\n")
        #log_tab4.insert("end", "Calibration\n")
        #send_relay("ON1")
        #send_stepper(TOPIC_STEP2, "forward")
        #send_stepper(TOPIC_STEP1, "forward")
        #time.sleep(60)
        #send_stepper(TOPIC_STEP2, "stop")
        #send_stepper(TOPIC_STEP1, "stop")

        log_tab4.insert("end", "=== AUTO END ===\n")
        auto_running = False

    threading.Thread(target=worker, daemon=True).start()
# ================= GUI =================
root = tk.Tk()
root.title("hello gigin – ESP32 Manager")
root.geometry("1100x650")
root.attributes("-topmost", True)
style = ttk.Style()
style.configure(
    "Small.TButton",
    font=("Segoe UI", 7,"bold"),   # ukuran huruf
    foreground="red",
    padding=(0, 0)          # tinggi tombol
)

notebook = ttk.Notebook(root)
tab1 = ttk.Frame(notebook)
tab4 = ttk.Frame(notebook)
notebook.add(tab1, text="MQTT + Control")
notebook.add(tab4, text="Auto Sequence")
notebook.pack(fill="both", expand=True)

# -------- TAB 1 --------
left = ttk.Frame(tab1)
left.pack(side="left", fill="y", padx=5)

right = ttk.Frame(tab1)
right.pack(side="left", fill="both", expand=True, padx=10)

ttk.Button(left, text="Connect MQTT", style="Small.TButton" ,command=start_mqtt).pack(fill="x")
ttk.Button(left, text="Stop MQTT",style="Small.TButton" , command=stop_mqtt).pack(fill="x", pady=2)

# ===== ESP32-A SENSOR =====
gA = ttk.LabelFrame(left, text="ESP32-A Air Quality")
gA.pack(fill="x", pady=1)

tempA_var=tk.StringVar(value="-")
humA_var=tk.StringVar(value="-")
pm25A_var=tk.StringVar(value="-")
co2A_var=tk.StringVar(value="-")
pressA_var=tk.StringVar(value="-")

labels = [
    ("Temp", tempA_var),
    ("Hum", humA_var),
    ("PM2.5", pm25A_var),
    ("CO2", co2A_var),
    ("Press", pressA_var),
]
for i,(t,v) in enumerate(labels):
    ttk.Label(gA,text=t).grid(row=i,column=0,sticky="w")
    ttk.Label(gA,textvariable=v).grid(row=i,column=1,sticky="w")

# ===== ESP32-B STEPPER =====
g1 = ttk.LabelFrame(left, text="Stepper 1")
g1.pack(fill="x", pady=1)
for txt,cmd in [("FORWARD","forward"),("BACKWARD","backward"),("STOP","stop")]:
    ttk.Button(g1,text=txt,style="Small.TButton" ,command=lambda c=cmd: send_stepper(TOPIC_STEP1,c)).pack(fill="x")

g2 = ttk.LabelFrame(left, text="Stepper 2")
g2.pack(fill="x", pady=1)
for txt,cmd in [("FORWARD","forward"),("BACKWARD","backward"),("STOP","stop")]:
    ttk.Button(g2,text=txt,command=lambda c=cmd: send_stepper(TOPIC_STEP2,c)).pack(fill="x")

# ===== RELAY =====
relay = ttk.LabelFrame(left, text="Relay")
relay.pack(fill="x", pady=1)
ttk.Label(relay,style="Small.TButton" , text="Pompa").pack(fill="x", pady=(0, 1))
ttk.Button(relay, text="OFF1",  command=lambda: send_relay("ON1")).pack(fill="x")
ttk.Button(relay, text="ON1", command=lambda: send_relay("OFF1")).pack(fill="x")
ttk.Label(relay,style="Small.TButton" , text="Vent Valve").pack(fill="x", pady=(1, 1))
ttk.Button(relay, text="CLOSE2", style="Small.TButton" , command=lambda: send_relay("ON2")).pack(fill="x")
ttk.Button(relay, text="OPEN2", style="Small.TButton" ,command=lambda: send_relay("OFF2")).pack(fill="x")
ttk.Label(relay,style="Small.TButton" , text="Valve 1").pack(fill="x", pady=(0, 1))
ttk.Button(relay, text="CLOSE3",  command=lambda: send_relay("ON3")).pack(fill="x")
ttk.Button(relay, text="OPEN3", command=lambda: send_relay("OFF3")).pack(fill="x", pady=(0, 5))

ttk.Button(relay, text="SAVE MANUAL", style="Small.TButton",command=lambda: save_calibration_time_60s()).pack(fill="x")


#txt_log = tk.Text(right, height=25, state="disabled")
#txt_log.pack(fill="both", expand=True)

# Scrollbar vertikal
scrollbar = tk.Scrollbar(right)
scrollbar.pack(side="right", fill="y")

# Text widget dengan scrollbar
txt_log = tk.Text(right, height=25, state="disabled", yscrollcommand=scrollbar.set)
txt_log.pack(side="left", fill="both", expand=True)

# Hubungkan scrollbar dengan Text
scrollbar.config(command=txt_log.yview)

# Fungsi log yang bisa update Text walau state=disabled
def log(msg):
    txt_log.config(state="normal")
    txt_log.insert("end", msg + "\n")
    txt_log.see("end")  # auto scroll ke bawah
    txt_log.config(state="disabled")

# Bind mouse wheel (Windows & Linux)
txt_log.bind("<MouseWheel>", lambda e: txt_log.yview_scroll(-1*(e.delta//120), "units"))  # Windows
txt_log.bind("<Button-4>", lambda e: txt_log.yview_scroll(-1, "units"))  # Linux scroll up
txt_log.bind("<Button-5>", lambda e: txt_log.yview_scroll(1, "units"))   # Linux scroll down



# -------- TAB 4 --------
#ttk.Button(tab4, text="JALANKAN AUTO SEQUENCE", command=run_sequence).pack(pady=1)
frame_btn = ttk.Frame(tab4)
frame_btn.pack(pady=1)
ttk.Button(frame_btn,text="JALANKAN AUTO SEQUENCE", command=run_sequence).pack(side="left")
ttk.Label(frame_btn,text="V1 CLOSE").pack(side="right", padx=1)

frame_btn = ttk.Frame(tab4)
frame_btn.pack(pady=1)
ttk.Button(frame_btn,text="JALANKAN AUTO SEQUENCE2", command=run_sequence2).pack(side="left")
ttk.Label(frame_btn,text="V1 CLOSE >> V1 OPEN").pack(side="right", padx=1)

ttk.Button(tab4, text="JALANKAN AUTO SEQUENCE3", command=run_sequence3).pack(pady=1)
ttk.Button(tab4, text="JALANKAN AUTO SEQUENCE4", command=run_sequence4).pack(pady=1)
# ===== ESP32-A di TAB 4 =====
gA_tab4 = ttk.LabelFrame(tab4, text="ESP32-A Air Quality (Monitor)")
gA_tab4.pack(fill="x", pady=5, padx=10)

labels_tab4 = [
    ("Temp", tempA_var),
    ("Hum", humA_var),
    ("PM2.5", pm25A_var),
    ("CO2", co2A_var),
    ("Press", pressA_var),
]

for i, (t, v) in enumerate(labels_tab4):
    ttk.Label(gA_tab4, text=t).grid(row=i, column=0, sticky="w", padx=5)
    ttk.Label(gA_tab4, textvariable=v).grid(row=i, column=1, sticky="w", padx=5)
log_tab4 = tk.Text(tab4, height=25)
log_tab4.pack(fill="both", expand=True, padx=10, pady=10)




# ================= START =================
root.after(100, process_log)
threading.Thread(target=start_mqtt, daemon=True).start()
root.mainloop()
