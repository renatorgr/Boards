# v4_UM982_TM171_GNHPR_INHPR

Firmware Teensy 4.1 para AgOpenWeb — UM982 (duas antenas) **ou** u-blox F9P / X20P (uma antena) + TM171, Cytron ou Keya.
Envia o "conjunto padrão" de frases NMEA, que o AgOpenWeb junta numa só posição por época.

**Precisa de um AgOpenWeb com o PR #300** ("standard set" / epoch assembler, `develop` a partir da noite de 9 out 2026). Numa build mais antiga estas frases aparecem como "não aceites".

## Entradas

- Recetor GNSS na Serial7, 460800, 10 Hz. O tipo é detetado sozinho:
  - **UM982 (duas antenas):** GGA + VTG + HPR (GP ou GN). O KSXT já não é usado (pode ficar ligado, é ignorado).
  - **u-blox F9P / X20P (uma antena):** GGA + VTG. Sem HPR, o firmware não espera por ele: cada época sai logo que chegam o GGA e o VTG.
  - O GGA é obrigatório (marca cada época). Um VTG ou HPR conta como "enviado pelo recetor" se chegou nos últimos 3 s (`RECEIVER_SEEN_MS`); nos primeiros 3 s depois do arranque espera-se pelos dois.
- TM171 (Serial5, 115200): roll, pitch e yaw.

## Saída (UDP 9999): 4 linhas por época, uma por datagrama

| Linha | Conteúdo | O AgOpenWeb usa para |
|---|---|---|
| `$GNGGA` | GGA do UM982 (com `00` satélites / HDOP `9999.0` substituídos pelos últimos bons até 2 s) | posição, fix, satélites, HDOP, idade das correções |
| `$GNVTG` | velocidade e rumo do VTG do UM982; se o VTG vier vazio, calculados das posições GGA | velocidade |
| `$GNTHS` | heading das duas antenas (o valor do HPR do UM982, igual carácter a carácter); modo `A` = válido, `V` = sem heading dual | heading quando há duas antenas |
| `$INHPR` | TM171 (talker `IN` = sensor inercial): heading alinhado com o heading dual, **roll no campo pitch** (convenção do HPR), pitch no campo roll; QF 4 = TM171 OK, QF 0 = TM171 perdido | roll sempre; heading quando não há duas antenas |

Resultado no AgOpenWeb (verificado com o próprio código do AgOpenWeb):

| Situação | Heading | Roll |
|---|---|---|
| Duas antenas (HPR QF 4 fixo, ou 5 float) | `$GNTHS` (antenas) | TM171 |
| Segunda antena perdida / sem heading | TM171, com a fusão de uma antena do AgOpenWeb (fix-to-fix + IMU, marcha-atrás) | TM171 |
| TM171 perdido | antenas (ou fix-to-fix sem duas antenas) | 0 |

A barra de estado do AgOpenWeb mostra a família **`GGA+VTG+HPR+THS`** (o "HPR" é o `$INHPR` do TM171).

### Porque é que o heading dual vai em `$GNTHS` e não em `$GNHPR`

O AgOpenWeb começa uma época nova quando um tipo de frase se repete. Um `$GNHPR` (antenas) e um `$INHPR` (TM171) são os dois "HPR", por isso nunca ficariam na mesma posição: o `$INHPR` abria uma época sem GGA e era deitado fora. O `$GNTHS` leva o mesmo heading do HPR do UM982 e pode estar na mesma época que o `$INHPR`. O firmware continua a **ler** o `$GNHPR` do UM982.

## Uma antena: u-blox F9P / X20P

- No u-center: UART1 a 460800, GGA + VTG a 10 Hz (período de medição 100 ms), NMEA de alta precisão ligado, Compatibility mode e Limit82 desligados, entrada RTCM3 na UART1, outras mensagens desligadas. Versão NMEA: qualquer (4.11 recomendada). Guardar na memória do recetor.
- No AgOpenWeb: **"Dual GPS" desligado**. O `$GNTHS` vai sempre com `V`; o AgOpenWeb usa o heading do TM171 com a sua fusão de uma antena (fix-to-fix + IMU, marcha-atrás), como o `$PANDA`. Altura e posição da antena no perfil do veículo.
- Sentido do yaw do TM171: sem duas antenas o firmware não o consegue aprender e usa +1 (como o firmware oficial da AiO, que usa o yaw do TM171 diretamente). Se o heading rodar ao contrário nas curvas, pôr `TM171_YAW_SIGN -1`.
- Mensagem `[status] Single antenna: no HPR from the receiver ...` confirma o modo.

## Configuração do UM982

No UPrecise (ou por comandos), na porta ligada ao Teensy, a 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, e guardar (`SAVECONFIG`). Manter `CONFIG HEADING OFFSET` como estava (o heading do HPR já vem com esse offset aplicado).

## No AgOpenWeb

- **"Dual GPS" ligado** (desligado, o AgOpenWeb ignora o heading das antenas e usa só o TM171 + fix-to-fix).
- Orientação das antenas: `CONFIG HEADING OFFSET` no UM982 **ou** `DualHeadingOffset` no AgOpenWeb — só um dos dois.
- Sentido / zero do roll: "Roll invert" e "Roll zero" do AgOpenWeb (aplicam-se ao roll do `$INHPR`).
- `MinFixQuality` (por defeito 4 = RTK fixo): abaixo disso o fix é marcado inválido. Em bancada pôr 1 ou 2.
- Agora o HDOP e a idade das correções voltam a aparecer (vêm do GGA).

## Ajustes (topo de `zHandlers.ino`)

- `HPR_ACCEPT_FLOAT` 1 = heading dual também com HPR QF 5 (float); 0 = só QF 4 (fixo).
- `EPOCH_WAIT_MS` 60: o firmware envia a época logo que chegam o VTG e o HPR com a mesma hora do GGA; se um faltar, envia 60 ms depois do GGA. Uma frase que o recetor não envia de todo (HPR num F9P / X20P) não é esperada (`RECEIVER_SEEN_MS` 3000).
- `TM171_SWAP_ROLL_PITCH`: TM171 rodado 90° na placa ("Use Y axis" também troca roll/pitch).
- `TM171_YAW_SIGN`: 0 = sentido do yaw aprendido contra o heading dual (só com rotação real do conjunto); +1/-1 força-o.
- `GGA_HOLD_MS`: tempo máximo a repetir satélites/HDOP bons quando o UM982 manda `00` / `9999.0`.
- Diagnóstico (pôr a 0 no trator):
  - `FUSION_DEBUG` (1): uma linha `[fusion] DUAL ...` ou `[fusion] SINGLE ...` por segundo: QF e heading do HPR, `antRoll` (roll medido pelas antenas, para comparar com o do TM171), alinhamento do TM171, heading e roll enviados, velocidade em km/h e de onde vem (`vtg` ou `positions`).
  - `RAW_NMEA_DEBUG` (0): cada GGA, VTG e HPR tal como o UM982 os envia, com `[raw]`.
  - `SEND_TO_USB` (1): as 4 linhas enviadas ao AgOpenWeb aparecem também no monitor série.

## Mensagens de estado (`zStatus.ino`)

Linhas `[status] ...` (em inglês), **só quando algo muda** (o novo estado tem de durar 1 s):

- Ethernet: cabo ligado (IP do módulo e destino) / cabo desligado
- AgOpenWeb a comunicar (IP do PC) / sem comunicar há 5 s
- Correções RTCM a chegar do AgOpenWeb / pararam há 5 s
- GPS: primeira posição; mudanças de fix com satélites e idade das correções; sem GGA há 5 s; VTG a chegar / sem VTG (velocidade calculada das posições)
- Dual: heading das duas antenas OK (QF 4 fixo / 5 float, enviado em `$GNTHS`) / sem heading dual
- Single antenna: o recetor não envia HPR (F9P / X20P, ou HPR desligado no UM982)
- TM171: dados OK / sem dados

LEDs: verde = heading dual, vermelho = heading do TM171, vermelho a piscar = TM171 perdido.
