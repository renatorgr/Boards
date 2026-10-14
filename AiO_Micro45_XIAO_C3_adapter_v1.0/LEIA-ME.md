# Adaptador XIAO ESP32C3 para a AiO v4.5 Micro (encaixe GPS2 → Serial2)

Placa pequena (27,3 × 24,4 mm, 2 camadas) que encaixa no **encaixe do segundo GPS ("Dual-Left", U11)** da AiO v4.5 Micro, no lugar de um simpleRTK2B Micro, e liga um **Seeed XIAO ESP32C3** à **Serial2 do Teensy** (pinos 7 e 8).

## Ligações

| Pino do encaixe | Na AiO v4.5 Micro | No adaptador |
|---|---|---|
| 1 | 3,3 V (regulador K7803M de 1 A) | 3V3 do XIAO + C1 22 µF |
| 2 | Teensy pino 7 (RX2) | R1 100 Ω → D6 (TX, GPIO21) |
| 3 | Teensy pino 8 (TX2) | R2 100 Ω → D7 (RX, GPIO20) |
| 10, 11 | GND | GND |
| 6, 7, 8 | micro-USB do F9P | sem ligação |
| 16 | pino 12 do encaixe do GPS1 (só para dual F9P) | sem ligação |
| 18 | LED RTK_STAT_1 | sem ligação |
| restantes | — | sem ligação |

As posições dos 20 pinos foram copiadas do ficheiro PCB oficial da AiO v4.5 Micro (EasyEDA, 2024-02-05) e conferidas com o esquema: diferença 0,00 mm. O contorno fica dentro do desenho do módulo original na serigrafia da Micro, com o mesmo canto cortado (do lado do LED1 e da ficha RJ45).

## Ficheiros

- `AiO_Micro45_XIAO_C3.kicad_pro / .kicad_sch / .kicad_pcb`: projeto KiCad 7 (esquema + PCB), com as bibliotecas próprias (`AiO_XIAO_Adapter.pretty`, `AiO_XIAO_Adapter.kicad_sym`).
- `fabricacao_JLCPCB/Gerber_AiO_Micro45_XIAO_C3.zip`: Gerbers + furação, prontos para a JLCPCB.
- `fabricacao_JLCPCB/BOM_…csv` e `CPL_…csv`: para a montagem SMT da JLCPCB (só R1, R2, C1).
- `documentacao/`: esquema em PDF, vistas da placa e a posição na Micro.

## Abrir no EasyEDA

EasyEDA Pro: **File → Import → KiCad**, escolher a pasta do projeto (ou o .zip). Importa o esquema e o PCB.
Para encomendar não é preciso o EasyEDA: o zip dos Gerbers vai diretamente para a JLCPCB.

## Encomenda (JLCPCB)

- Gerber: `Gerber_AiO_Micro45_XIAO_C3.zip`. 2 camadas, 1,6 mm, qualquer cor. Os valores por defeito servem (pista mínima 0,3 mm, furo mínimo 0,3 mm).
- Montagem SMT (opcional): BOM + CPL. São só três peças 0805 e soldam-se bem à mão.

| Ref. | Peça | LCSC |
|---|---|---|
| R1, R2 | 100 Ω 1 % 0805 | C17408 |
| C1 | 22 µF 0805 X5R | C45783 |
| J1 | 2 × barra macho 1×10, **passo 2,00 mm** | (soldar à mão) |
| U1 | Seeed XIAO ESP32C3 | (soldar à mão) |

## Montagem

1. R1, R2 e C1 (se não vierem montados).
2. **As barras macho de 2,00 mm ficam por baixo**, com os pinos para baixo. Para ficarem bem alinhadas: enfia-as no encaixe GPS2 da Micro, pousa o adaptador por cima e solda-as por cima.
3. **XIAO por cima**, com o USB-C para o lado da marca "USB-C":
   - de preferência **soldado plano** (pelas meias-ilhas da borda): fica mais baixo, aguenta melhor a vibração e não deixa pontas por baixo;
   - se usares os pinos que vêm com o XIAO, **corta as pontas rentes por baixo**. Por baixo do adaptador, na Micro, está a resistência R31, mesmo debaixo da fila D0–D6.
4. Orientação na Micro: o canto cortado do adaptador coincide com o canto cortado desenhado na serigrafia do encaixe GPS2, e o pino 1 (ilha quadrada, ponto na serigrafia) fica junto ao pino 1 do encaixe.

## Cuidados

- O XIAO é alimentado pelo pino 3V3 (3,3 V da placa). **Não ligues o USB-C do XIAO com o adaptador montado na Micro.** Programa-o fora da placa, ou por WiFi (OTA) depois da primeira vez.
- As caixas Hammond 1455 são de alumínio: usa uma antena WiFi externa no conector U.FL do XIAO, levada para fora pela tampa.
- O encaixe GPS2 deixa de estar disponível para um segundo F9P (não é preciso com o UM982).

## Firmware

- Teensy: `Serial2.begin(460800);` (pinos 7 RX2 e 8 TX2). O firmware v6 ainda não usa a Serial2.
- ESP32-C3 (Arduino): UART0 nos pinos D7 = GPIO20 (RX) e D6 = GPIO21 (TX), por exemplo `Serial0.begin(460800);`, ou `Serial1.begin(460800, SERIAL_8N1, 20, 21);`.
- Ao arrancar, o ESP32-C3 escreve umas linhas de arranque a 115200 no GPIO21: o Teensy recebe lixo durante meio segundo, e o código tem de o ignorar.
- A v5 oficial (Firmware_Teensy_AiO_26) usa exatamente esta ligação (Serial2, 460800) para o seu ESP32. Ver `docs/ESP32_BRIDGE_INTERFACE.md` nesse repositório.

## Verificações feitas

- O esquema e o PCB têm as mesmas ligações (netlist exportada do esquema = ligações do PCB).
- DRC do KiCad 7: 0 erros, 0 ligações em falta.
- Pinos e ligações conferidos com o esquema e o PCB oficiais da AiO v4.5 Micro (AgOpenGPS-Official/Boards).
- Pegada do XIAO conforme a biblioteca KiCad oficial da Seeed (XIAO-ESP32-C3-DIP-SMD).
- Ainda não foi fabricado nem testado numa placa real.
