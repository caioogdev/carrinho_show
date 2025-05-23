# Projeto: Detecção de Cor e Controle por Wi-Fi com ESP32-CAM e OpenCV

Este repositório reúne um conjunto de scripts para controle e análise visual via Wi-Fi utilizando uma câmera ESP32-CAM. O projeto combina visão computacional com filtros de imagem e classificação de cor em tempo real usando Python (OpenCV), além de um sketch Arduino que permite controle remoto de um carrinho via ESP32.

## Estrutura do Projeto

### 1. `camera 2 wifi filtro.py`
Script em Python para captura de vídeo da ESP32-CAM via Wi-Fi. Ele aplica diferentes filtros de imagem (Gaussian Blur, Bilateral, CLAHE, etc.) e realiza a detecção da cor predominante em tempo real com base no componente Hue (matiz) do modelo HSV.

### 2. `LFR_Demo - wifi cor ok copy com filtros teste copy funcinando cor wifi.py`
Versão mais elaborada e funcional do script de captura e detecção de cor. Realiza:
- Comparativo visual dos filtros aplicados
- Buffer circular para média temporal dos frames
- Classificação da cor predominante (vermelho, verde, azul)

### 3. `Carrinho_controle_xbox_2025_RGB.ino-wifi.ino`
Sketch em C++ para ESP32 que implementa o controle de um carrinho com comandos via Wi-Fi, permitindo movimentação e possível integração com controle via joystick ou comandos remotos.

## Requisitos

### Python
- Python 3.7+
- Bibliotecas:
  - `opencv-python`
  - `numpy`
  - `requests`

Instalação recomendada:

```bash
pip install -r requirements.txt
