import cv2
import cv2.aruco as aruco
from pymodbus.client import ModbusSerialClient
import time

SLAVE_GRIPPER = 10
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200      
TARGET_ARUCO_ID = 30

# alamat coil gripper
COIL_ADDRESS = 0

def send_command(client, command):
    """
    kirim command boolean ke gripper slave 
    command: True = BUKA, False = TUTUP
    """
    try:
        result = client.write_coil(COIL_ADDRESS, command, slave=SLAVE_GRIPPER)
        
        if result.isError():
            print(f"[ERROR] Gagal kirim command {command}: {result}")
            return False
        
        status = "BUKA" if command else "TUTUP"
        print(f"[INFO] Coil {COIL_ADDRESS} = {command} ({status}) terkirim ke slave {SLAVE_GRIPPER}")
        return True
    except Exception as e:
        print(f"[EXCEPTION] {e}")
        return False



# deteksi aruco
def findAruco(img, marker_size=6, total_markers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    detector = aruco.ArucoDetector(arucoDict)
    bbox, ids, rejected = detector.detectMarkers(gray)
    
    if ids is not None:
        print("Aruco terdeteksi:", ids.flatten())
    
    if draw and ids is not None:
        aruco.drawDetectedMarkers(img, bbox, ids)
    
    return bbox, ids


def main():
    # inisialisasi Modbus client
    client = ModbusSerialClient(
        method='rtu',
        port=PORT,
        baudrate=BAUDRATE,
        timeout=1,
        bytesize=8,
        parity='N',
        stopbits=1
    )
    
    if not client.connect():
        print("Gagal konek ke Modbus!")
        return
    
    print("Modbus terhubung. Memulai deteksi Aruco...")
    print(f"[INFO] Target Aruco ID: {TARGET_ARUCO_ID}")
    print(f"[INFO] Coil Address: {COIL_ADDRESS}, Slave ID: {SLAVE_GRIPPER}")
    
    # inisialisasi kamera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Gagal buka kamera!")
        client.close()
        return
    
    gripper_activated = False
    
    try:
        while True:
            ret, img = cap.read()
            if not ret:
                print("Camera error")
                break
            
            bbox, ids = findAruco(img)
            
            target_detected = False
            
            if ids is not None:
                detected_ids = ids.flatten()
                if TARGET_ARUCO_ID in detected_ids:
                    target_detected = True
                    print(f">>> TARGET ARUCO ID {TARGET_ARUCO_ID} TERDETEKSI!")
            
            if target_detected:
                if not gripper_activated:
                    print(">>> Mengirim perintah BUKA (True) ke coil...")
                    send_command(client, True)  
                    gripper_activated = True
                    
                    # tunggu sebentar
                    time.sleep(1.5)
                    
            else:
                if gripper_activated:
                    print(">>> Target hilang, mengirim perintah TUTUP (False)...")
                    send_command(client, False)
                    gripper_activated = False
            
            # tampilkan gambar
            cv2.imshow("Aruco Detection", img)
            
            if cv2.waitKey(1) == ord('q'):
                break
                
    finally:
        # make sure gripper ditutup saat program berhenti
        print("\n>>> Mengirim perintah TUTUP (False) sebelum keluar...")
        send_command(client, False)
        
        cap.release()
        cv2.destroyAllWindows()
        client.close()
        print("Program selesai")

if __name__ == "__main__":
    main()
