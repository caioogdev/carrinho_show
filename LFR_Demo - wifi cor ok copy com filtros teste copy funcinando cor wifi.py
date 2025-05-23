import cv2
import numpy as np
import time
import requests
import socket
from collections import deque

# === CONFIGURAÇÕES DE FPS E BUFFER ===
FPS = 10
SEGUNDOS_ANALISE = 2
MAX_FRAMES = FPS * SEGUNDOS_ANALISE

buffers = {}
ultima_cor = {}
filtros_aplicados = []
fundo_gaussiano = None  # Média dos primeiros frames do filtro Gaussian

# === INICIALIZAÇÃO DO VÍDEO ===
url = 'http://192.168.118.81:81/stream'  # IP da ESP32-CAM
cap = cv2.VideoCapture(url)
cap.set(3, 640)
cap.set(4, 480)

# === AJUSTES DE CÂMERA ===
time.sleep(1)
try:
    requests.get('http://192.168.118.81/control?var=awb_gain&val=1')
    requests.get('http://192.168.118.81/control?var=wb_mode&val=4')
except:
    pass

# === CONFIGURAÇÃO DO SOCKET TCP PARA O ESP32 (carrinho) ===
ESP32_IP = '192.168.118.83'
ESP32_PORTA = 12345

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    tcp_socket.connect((ESP32_IP, ESP32_PORTA))
    print("✅ Conectado ao ESP32 com sucesso!")
except Exception as e:
    print(f"❌ Erro ao conectar ao ESP32: {e}")
    tcp_socket = None

# === CLASSIFICAÇÃO SIMPLIFICADA ===
def classificar_cor(h):
    if h < 18 or h >= 160:
        return 'Vermelho'
    elif 35 <= h < 85:
        return 'Verde'
    elif 85 <= h < 130:
        return 'Azul'
    else:
        return None

# === CLAHE CONFIG ===
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# === LOOP PRINCIPAL ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Falha ao capturar o frame")
        break

    hsv_base = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    visoes = []

    filtros = {
        'Original': frame,
        'Gaussian': cv2.GaussianBlur(frame, (5, 5), 0),
        'Bilateral': cv2.bilateralFilter(frame, 9, 75, 75),
        'Sobel': cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=5)),
        'Equalized': cv2.cvtColor(cv2.equalizeHist(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)), cv2.COLOR_GRAY2BGR),
        'CLAHE': cv2.cvtColor(clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)), cv2.COLOR_GRAY2BGR),
        'Saturação++': cv2.cvtColor(np.clip(hsv_base * np.array([1, 1.5, 1]), 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR),
        'HSV Mask': cv2.inRange(hsv_base, (0, 100, 100), (180, 255, 255))
    }

    for nome, imagem in filtros.items():
        if nome == 'HSV Mask':
            imagem_visivel = cv2.cvtColor(imagem, cv2.COLOR_GRAY2BGR)
            hsv = hsv_base
        elif nome == 'Gaussian':
            altura = imagem.shape[0]
            corte = int(altura * 0.35)
            imagem_crop = imagem[corte:, :]
            hsv = cv2.cvtColor(imagem_crop, cv2.COLOR_BGR2HSV)

            # Preenche o topo para manter altura original
            preenchimento = np.zeros((corte, imagem.shape[1], 3), dtype=np.uint8)
            imagem_visivel = np.vstack([preenchimento, imagem_crop])
        else:
            imagem_visivel = imagem
            hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)

        media_frame = np.mean(hsv.reshape(-1, 3), axis=0)
        if nome not in buffers:
            buffers[nome] = deque(maxlen=MAX_FRAMES)
            ultima_cor[nome] = None
        buffers[nome].append(media_frame)

        texto = ""

        if len(buffers[nome]) == MAX_FRAMES:
            media_geral = np.mean(buffers[nome], axis=0).astype(int)
            h, s, v = media_geral

            if nome == 'Gaussian':
                if fundo_gaussiano is None:
                    fundo_gaussiano = media_geral
                    print(f"[{nome}] Fundo capturado: H:{fundo_gaussiano[0]}")
                    cor = None
                else:
                    h_diferenca = abs(int(h) - int(fundo_gaussiano[0]))
                    limiar = 10
                    cor = classificar_cor(h) if h_diferenca > limiar else None

                print(f"[{nome}] Ação: {cor if cor else 'Nenhuma'}")
                texto = f"[{nome}] Cor: {cor if cor else 'Nenhuma'} (H:{h})"

                if tcp_socket:
                    try:
                        if cor == "Vermelho":
                            comando = "Vire à esquerda\n"
                        elif cor == "Verde":
                            comando = "Siga eem frente\n"
                        elif cor == "Azul":
                            comando = "Vire à direita\n"
                        else:
                            comando = "Parar\n"

                        tcp_socket.sendall(comando.encode())
                        print(f"📤 Enviado ao ESP32: {comando.strip()}")
                    except Exception as e:
                        print(f"❌ Erro ao enviar comando: {e}")

                ultima_cor[nome] = cor
            else:
                cor = classificar_cor(h)
                ultima_cor[nome] = cor
                texto = f"[{nome}] Cor: {cor if cor else 'Nenhuma'} (H:{h})"
        else:
            texto = f"[{nome}] Aguardando... ({len(buffers[nome])}/{MAX_FRAMES})"

        img_texto = imagem_visivel.copy()
        cv2.putText(img_texto, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        visoes.append(img_texto)

    bloco1 = cv2.hconcat(visoes[:4])
    bloco2 = cv2.hconcat(visoes[4:]) if len(visoes) > 4 else np.zeros_like(bloco1)
    saida = cv2.vconcat([bloco1, bloco2])

    cv2.imshow("Comparativo Filtros - Detecção de Cor", saida)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if tcp_socket:
    tcp_socket.close()
