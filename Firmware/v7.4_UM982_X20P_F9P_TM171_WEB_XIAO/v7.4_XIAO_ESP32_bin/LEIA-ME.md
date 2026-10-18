# Programa do XIAO já compilado (v7.4)

O mesmo programa (`v7.4_XIAO_ESP32`), compilado para os dois XIAO. O ESP32C3 e o ESP32S3 são chips diferentes: cada um precisa do seu ficheiro.

| Ficheiro | Para quê |
|---|---|
| `v7.4_XIAO_ESP32C3.bin` | XIAO ESP32C3: atualizar pelo WiFi (página do XIAO → Programa) |
| `v7.4_XIAO_ESP32S3.bin` | XIAO ESP32S3: atualizar pelo WiFi (página do XIAO → Programa) |
| `v7.4_XIAO_ESP32C3_USB.bin` | XIAO ESP32C3: primeira instalação pelo USB-C, no endereço 0x0 |
| `v7.4_XIAO_ESP32S3_USB.bin` | XIAO ESP32S3: primeira instalação pelo USB-C, no endereço 0x0 |

Se escolheres o ficheiro do outro XIAO, a página recusa-o antes de o enviar.

## Primeira instalação pelo USB-C, sem o Arduino IDE

No computador, com o Chrome ou o Edge:

1. XIAO **fora da Micro**, ligado ao computador pelo USB-C.
2. Abre https://espressif.github.io/esptool-js/
3. Baudrate 921600 → **Connect** → escolhe a porta do XIAO.
   - Se a porta não aparecer: desliga o cabo, carrega no botão **BOOT** do XIAO, liga o cabo com o BOOT carregado e larga-o.
4. Em **Flash Address** escreve `0x0` (a página pode trazer outro valor) e escolhe o ficheiro `…_USB.bin` do teu XIAO (C3 ou S3).
5. **Program**. Quando acabar, carrega no botão **RESET** do XIAO (ou desliga e volta a ligar o cabo).
6. O XIAO cria a rede `XIAO-xxxx`, palavra-passe `agopengps`.

Também se pode instalar pelo Arduino IDE, a compilar o programa (ver o README do programa).

## Depois: atualizar pelo WiFi

Telemóvel ligado à rede do XIAO → página → **Programa** → escolher o `.bin` **sem** "_USB" do teu XIAO → palavra-passe → **Instalar**. As definições do WiFi ficam.

## Licença

GNU GPL v3 ou posterior (ficheiro `COPYING` na pasta do programa `v7.4_XIAO_ESP32`). Programa escrito por Claude (Anthropic) para o renatorgr; a ligação ao Teensy usa o protocolo da ponte ESP32 do firmware oficial v26 do AgOpenGPS.
