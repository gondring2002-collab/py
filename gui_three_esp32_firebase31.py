import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import json
import time
import datetime
import paho.mqtt.client as mqtt

# Optional: Firebase admin
try:
    import firebase_admin
    from firebase_admin import credentials, db
    FIREBASE_AVAILABLE = True
except Exception:
    FIREBASE_AVAILABLE = False

# ---------------- CONFIG ----------------
BROKER = "test.mosquitto.org"
TOPIC_A_SENSOR = "sensor/airquality/A"
TOPIC_C_SENSOR = "sensor/airquality/C"
TOPIC_STEPPER  = "home/steppermotor"
TOPIC_RELAY_B  = "home/esp32b/relay"
TOPIC_OTA_A = "esp32/A/ota"
TOPIC_OTA_B = "esp32/B/ota"
TOPIC_OTA_STATUS_A = "esp32/A/ota/status"
TOPIC_OTA_STATUS_B = "esp32/B/ota/status"

SERVICE_ACCOUNT_FILE = "iot-airquality-9f42d-firebase-adminsdk-fbsvc-34523fc412.json"
DATABASE_URL = "https://iot-airquality-9f42d-default-rtdb.asia-southeast1.firebasedatabase.app/"

FIREBASE_PATH_A = "AirQuality/ESP32A"
FIREBASE_PATH_C = "AirQuality/ESP32C"
DEFAULT_OTA_TOKEN = "MY_SECRET_TOKEN"

client = mqtt.Client()
latest_A = None
latest_C = None
stepper_history = []

# ---------------- Firebase init ----------------
if FIREBASE_AVAILABLE:
    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_FILE)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        ref_A = db.reference(FIREBASE_PATH_A)
        ref_C = db.reference(FIREBASE_PATH_C)
    except Exception as e:
        print("Firebase init error:", e)
        FIREBASE_AVAILABLE = False

# ---------------- MQTT CALLBACKS ----------------
def mqtt_on_connect(client, userdata, flags, rc):
    if rc == 0:
        log_tab1("MQTT connected")
        client.subscribe(TOPIC_A_SENSOR)
        client.subscribe(TOPIC_C_SENSOR)
        client.subscribe(TOPIC_OTA_STATUS_A)
        client.subscribe(TOPIC_OTA_STATUS_B)
    else:
        log_tab1(f"MQTT connect failed rc={rc}")

def mqtt_on_message(client, userdata, msg):
    global latest_A, latest_C
    topic = msg.topic
    payload = msg.payload.decode(errors="ignore")
    try:
        data = json.loads(payload)
    except:
        data = None

    if topic == TOPIC_A_SENSOR:
        if isinstance(data, dict):
            latest_A = data
            update_sensor_display("A", data)
        else:
            log_tab1(f"Malformed JSON from A: {payload}")

    elif topic == TOPIC_C_SENSOR:
        if isinstance(data, dict):
            latest_C = data
            update_sensor_display("C", data)
        else:
            log_tab1(f"Malformed JSON from C: {payload}")

    elif topic in (TOPIC_OTA_STATUS_A, TOPIC_OTA_STATUS_B):
        log_tab1(f"OTA status {topic}: {payload}")

def start_mqtt():
    try:
        client.on_connect = mqtt_on_connect
        client.on_message = mqtt_on_message
        client.connect(BROKER, 1883, 60)
        client.loop_start()
    except Exception as e:
        messagebox.showerror("MQTT", str(e))

# ---------------- GUI HELPERS ----------------
def log_tab1(msg):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    txt_tab1.configure(state="normal")
    txt_tab1.insert("end", f"[{timestamp}] {msg}\n")
    txt_tab1.see("end")
    txt_tab1.configure(state="disabled")

def update_sensor_display(dev, data):
    if dev == "A":
        tempA_var.set(data.get("temp", "-"))
        humA_var.set(data.get("hum", "-"))
        pm25A_var.set(data.get("pm25", "-"))
        co2A_var.set(data.get("co2", "-"))
    else:
        tempC_var.set(data.get("temp", "-"))
        humC_var.set(data.get("hum", "-"))
        pm25C_var.set(data.get("pm25", "-"))
        co2C_var.set(data.get("co2", "-"))

# ---------------- STEPPER + RELAY B ----------------
def send_stepper_cmd(cmd):
    try:
        client.publish(TOPIC_STEPPER, cmd)
        log_tab1(f"Stepper cmd: {cmd}")
        t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        stepper_history.append({"ts": t, "cmd": cmd})
        history_tree.insert("", "end", values=(t, cmd))
    except Exception as e:
        log_tab1(str(e))

def send_relay(state):
    try:
        client.publish(TOPIC_RELAY_B, state)
        log_tab1(f"Relay B: {state}")
    except Exception as e:
        log_tab1(str(e))

def save_stepper_history_to_file():
    if not stepper_history:
        messagebox.showinfo("History", "Empty")
        return
    f = filedialog.asksaveasfilename(defaultextension=".txt")
    if not f:
        return
    with open(f, "w") as fp:
        for e in stepper_history:
            fp.write(f"{e['ts']} | {e['cmd']}\n")
    messagebox.showinfo("Saved", f)

# ---------------- FIREBASE ----------------
def save_now_to_firebase():
    if not FIREBASE_AVAILABLE:
        messagebox.showwarning("Firebase", "Not configured")
        return
    try:
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if latest_A: ref_A.child(now).set(latest_A)
        if latest_C: ref_C.child(now).set(latest_C)
        log_tab2(f"Saved manually to Firebase")
    except Exception as e:
        messagebox.showerror("Firebase", str(e))

def log_tab2(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    txt_tab2.configure(state="normal")
    txt_tab2.insert("end", f"[{ts}] {msg}\n")
    txt_tab2.see("end")
    txt_tab2.configure(state="disabled")

def periodic_hourly_save():
    """Menunggu jam berikutnya dan ambil 10 data tiap menit"""
    def worker():
        while True:
            now = datetime.datetime.now()
            next_hour = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)
            log_tab2(f"Menunggu auto-save jam berikutnya: {next_hour.strftime('%H:%M:%S')}")
            delta = (next_hour - now).total_seconds()
            time.sleep(delta)

            # ambil 10 data tiap menit
            for i in range(10):
                ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                if FIREBASE_AVAILABLE:
                    if latest_A: ref_A.child(ts).set(latest_A)
                    if latest_C: ref_C.child(ts).set(latest_C)
                log_tab2(f"[{i+1}/10] Auto-saved data at {ts}")
                time.sleep(60)  # tunggu 1 menit

    threading.Thread(target=worker, daemon=True).start()

# ---------------- OTA ----------------
def send_ota_command():
    device = ota_device_var.get()
    url = ota_url_var.get().strip()
    ver = ota_version_var.get().strip()
    token = ota_token_var.get().strip() or DEFAULT_OTA_TOKEN
    if not url:
        messagebox.showwarning("OTA", "URL kosong")
        return

    topic = TOPIC_OTA_A if device == "A" else TOPIC_OTA_B
    payload = {"cmd": "update", "url": url, "version": ver, "token": token}
    client.publish(topic, json.dumps(payload))
    log_tab1(f"OTA sent to {device}: {url}")

# ---------------- GUI ----------------
root = tk.Tk()
root.title("ESP32 Manager")
root.geometry("1000x700")

notebook = ttk.Notebook(root)
tab1 = ttk.Frame(notebook)
tab2 = ttk.Frame(notebook)
tab3 = ttk.Frame(notebook)
tab4 = ttk.Frame(notebook)
notebook.add(tab1, text="MQTT (A/C + B stepper)")
notebook.add(tab2, text="Firebase History")
notebook.add(tab3, text="OTA")
notebook.add(tab4, text="Auto Sequence")
notebook.pack(fill="both", expand=True)

# ---------------- TAB 1 ----------------
left1 = ttk.Frame(tab1)
left1.pack(side="left", fill="y", padx=10, pady=10)

lbl_mqtt = ttk.Label(left1, text="MQTT: " + BROKER)
lbl_mqtt.pack(anchor="w")
ttk.Button(left1, text="Connect MQTT",
           command=lambda: threading.Thread(target=start_mqtt, daemon=True).start()).pack(fill="x", pady=5)

# ESP32-A
gA = ttk.LabelFrame(left1, text="ESP32-A (Sensor)")
gA.pack(fill="x", pady=5)
tempA_var = tk.StringVar(value="-")
humA_var = tk.StringVar(value="-")
pm25A_var = tk.StringVar(value="-")
co2A_var = tk.StringVar(value="-")
ttk.Label(gA, text="Temp:").grid(row=0, column=0); ttk.Label(gA, textvariable=tempA_var).grid(row=0, column=1)
ttk.Label(gA, text="Hum:").grid(row=1, column=0); ttk.Label(gA, textvariable=humA_var).grid(row=1, column=1)
ttk.Label(gA, text="PM2.5:").grid(row=2, column=0); ttk.Label(gA, textvariable=pm25A_var).grid(row=2, column=1)
ttk.Label(gA, text="CO2:").grid(row=3, column=0); ttk.Label(gA, textvariable=co2A_var).grid(row=3, column=1)

# ESP32-C
gC = ttk.LabelFrame(left1, text="ESP32-C (Sensor)")
gC.pack(fill="x", pady=5)
tempC_var = tk.StringVar(value="-")
humC_var = tk.StringVar(value="-")
pm25C_var = tk.StringVar(value="-")
co2C_var = tk.StringVar(value="-")
ttk.Label(gC, text="Temp:").grid(row=0, column=0); ttk.Label(gC, textvariable=tempC_var).grid(row=0, column=1)
ttk.Label(gC, text="Hum:").grid(row=1, column=0); ttk.Label(gC, textvariable=humC_var).grid(row=1, column=1)
ttk.Label(gC, text="PM2.5:").grid(row=2, column=0); ttk.Label(gC, textvariable=pm25C_var).grid(row=2, column=1)
ttk.Label(gC, text="CO2:").grid(row=3, column=0); ttk.Label(gC, textvariable=co2C_var).grid(row=3, column=1)

# ESP32-B
gB = ttk.LabelFrame(left1, text="ESP32-B (Stepper)")
gB.pack(fill="x", pady=5)
ttk.Button(gB, text="FORWARD", command=lambda: send_stepper_cmd("forward")).pack(fill="x", pady=2)
ttk.Button(gB, text="BACKWARD", command=lambda: send_stepper_cmd("backward")).pack(fill="x", pady=2)
ttk.Button(gB, text="STOP", command=lambda: send_stepper_cmd("stop")).pack(fill="x", pady=2)
ttk.Button(gB, text="Relay ON", command=lambda: send_relay("on")).pack(fill="x", pady=2)
ttk.Button(gB, text="Relay OFF", command=lambda: send_relay("off")).pack(fill="x", pady=2)
ttk.Button(gB, text="Save Stepper History to File", command=save_stepper_history_to_file).pack(fill="x", pady=6)

txt_tab1 = tk.Text(tab1, height=15, state="disabled")
txt_tab1.pack(fill="both", expand=True, padx=10, pady=10)

# ---------------- TAB 2 ----------------
top2 = ttk.Frame(tab2)
top2.pack(fill="x", padx=10, pady=5)
ttk.Button(top2, text="Load A", command=lambda: load_history_from_firebase("A")).pack(side="left", padx=4)
ttk.Button(top2, text="Load C", command=lambda: load_history_from_firebase("C")).pack(side="left", padx=4)
ttk.Button(top2, text="Save Grafik", command=lambda: save_graph()).pack(side="left", padx=4)
ttk.Button(top2, text="Save Manual", command=save_now_to_firebase).pack(side="left", padx=4)

fb_cols = ("timestamp","temp","hum","co2","pm25")
fb_tree = ttk.Treeview(tab2, columns=fb_cols, show="headings")
for c in fb_cols:
    fb_tree.heading(c, text=c.upper())
fb_tree.pack(fill="both", expand=True, padx=10, pady=10)

txt_tab2 = tk.Text(tab2, height=8, state="disabled")
txt_tab2.pack(fill="both", expand=True, padx=10, pady=10)

# ---------------- TAB 3 ----------------
f3 = ttk.LabelFrame(tab3, text="OTA Control")
f3.pack(fill="x", padx=10, pady=10)
ota_device_var = tk.StringVar(value="A")
ttk.Label(f3, text="Device:").grid(row=0, column=0)
ttk.Combobox(f3, textvariable=ota_device_var, values=["A","B"], width=10, state="readonly").grid(row=0, column=1)
ota_url_var = tk.StringVar()
ttk.Label(f3, text="Firmware URL:").grid(row=1, column=0, columnspan=2)
ttk.Entry(f3, textvariable=ota_url_var, width=60).grid(row=2, column=0, columnspan=2, pady=3)
ota_version_var = tk.StringVar()
ttk.Label(f3, text="Version:").grid(row=3, column=0)
ttk.Entry(f3, textvariable=ota_version_var).grid(row=3, column=1)
ota_token_var = tk.StringVar(value=DEFAULT_OTA_TOKEN)
ttk.Label(f3, text="Token:").grid(row=4, column=0)
ttk.Entry(f3, textvariable=ota_token_var).grid(row=4, column=1)
ttk.Button(f3, text="SEND OTA", command=send_ota_command).grid(row=5, column=0, pady=6)

lf3 = ttk.LabelFrame(tab3, text="OTA / Log Window")
lf3.pack(fill="both", expand=True, padx=10, pady=10)
txt_tab3 = tk.Text(lf3, height=12)
txt_tab3.pack(fill="both", expand=True)

# ---------------- TAB 4 ----------------
log_tab4 = tk.Text(tab4, height=15)
log_tab4.pack(fill="both", expand=True, padx=10, pady=10)
def log4(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    log_tab4.insert("end", f"[{ts}] {msg}\n")
    log_tab4.see("end")

def run_sequence():
    def thread_func():
        log4("Mulai sequence")
        send_stepper_cmd("backward")
        log4("Stepper mundur")
        time.sleep(5)
        send_relay("on")
        log4("Relay ON selama 1 menit")
        for i in range(60):
            if latest_A: log4(f"ESP32-A: {latest_A}")
            if latest_C: log4(f"ESP32-C: {latest_C}")
            time.sleep(1)
        send_relay("off")
        log4("Relay OFF")
        log4("Sequence selesai")
    threading.Thread(target=thread_func, daemon=True).start()

ttk.Button(tab4, text="Jalankan", command=run_sequence).pack(pady=5)

# ---------------- START ----------------
threading.Thread(target=start_mqtt, daemon=True).start()
periodic_hourly_save()
root.mainloop()
