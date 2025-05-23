import cv2
import numpy as np
import time
import requests

FPS = 40
SEGUNDOS_ANALISE = 20

# === INICIALIZAÇÃO DO VÍDEO ===
url = 'http://192.168.118.33:81/stream'
cap = cv2.VideoCapture(url)
cap.set(3, 320)
cap.set(4, 240)

# === ENVIO DE COMANDOS ===
time.sleep(1)
try:
    #requests.get('http://192.168.4.100/control?var=wb_mode&val=3')
    #requests.get('http://192.168.4.100/control?var=saturation&val=4')
    requests.get('http://192.168.118.33/control?var=framesize&val=6')
except:
    pass

# === CLAHE CONFIG ===
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# === LOOP PRINCIPAL ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha ao capturar o frame")
        break

    hsv_base = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    visoes = []

    filtros = {
        'Original': frame,
        'Gaussian': cv2.GaussianBlur(frame, (5, 5), 0),
        'Sobel': cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=5)),
        'Equalized': cv2.cvtColor(cv2.equalizeHist(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)), cv2.COLOR_GRAY2BGR),
        'Laplaciano': cv2.cvtColor(cv2.convertScaleAbs(cv2.Laplacian(frame, cv2.CV_64F)), cv2.COLOR_BGR2RGB),
'Threshold Adapt': cv2.cvtColor(cv2.adaptiveThreshold(
    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
    255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV,
    11,
    2), cv2.COLOR_GRAY2BGR),

        'Contorno': cv2.cvtColor(cv2.Canny(frame, 100, 200), cv2.COLOR_GRAY2BGR),
        'HSV Mask': cv2.inRange(hsv_base, (0, 100, 100), (180, 255, 255))
    }

    for nome, imagem in filtros.items():
        if nome == 'HSV Mask':
            imagem_visivel = cv2.cvtColor(imagem, cv2.COLOR_GRAY2BGR)
        else:
            imagem_visivel = imagem

        texto = f"[{nome}]"
        img_texto = imagem_visivel.copy()
        cv2.putText(img_texto, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        visoes.append(img_texto)

    bloco1 = cv2.hconcat(visoes[:4])
    bloco2 = cv2.hconcat(visoes[4:]) if len(visoes) > 4 else np.zeros_like(bloco1)
    saida = cv2.vconcat([bloco1, bloco2])
    saida = cv2.resize(saida, None, fx=0.5, fy=0.5)

    cv2.imshow("Comparativo Filtros - Detecção de Contorno", saida)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
