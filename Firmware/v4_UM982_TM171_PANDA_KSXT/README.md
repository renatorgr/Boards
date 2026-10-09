# v4_UM982_TM171

Firmware Teensy 4.1 para AgOpenWeb — UM982 (duas antenas) + TM171, Cytron ou Keya.

**Precisa de um AgOpenWeb com suporte a `$KSXT`** (PR #288 do repositório oficial, `develop` de 9 out 2026 ou posterior).

## Entradas

- UM982 (Serial7, 460800): **GGA + KSXT a 10 Hz**. VTG e HPR não são usados (podem ser desligados no UM982). O GGA é obrigatório: o firmware envia uma frase por cada GGA.
- TM171 (Serial5, 115200): roll, pitch e yaw.

## Saída (UDP 9999, uma frase por GGA)

| Situação | Frase | Posição / velocidade | Heading | Roll |
|---|---|---|---|---|
| Dual: KSXT com qualidade do heading 2 (float) ou 3 (fixo), < 400 ms | `$KSXT` reencaminhado tal como vem do UM982 | do KSXT | do KSXT | do KSXT (linha das antenas; o AgOpenWeb só o usa com RTK fixo) |
| Uma antena (KSXT em falta, qualidade 0/1, ou parado) | `$PANDA` | GGA; velocidade do KSXT da mesma época, senão das posições GGA | TM171 alinhado ao último heading do KSXT (inteiro ×10) | TM171 (×10) |
| Uma antena e TM171 perdido | `$PANDA` | GGA | `65535` (sem IMU) → AgOpenWeb usa fix-to-fix | 0 |

Em dual o TM171 não vai para o AgOpenWeb: só serve para manter o seu yaw alinhado com o heading do KSXT, para o primeiro `$PANDA` sair sem salto. Com `$PANDA`, o AgOpenWeb faz a fusão de uma antena (heading fix-to-fix + IMU com desvio aprendido, peso `HeadingFusionWeight`, marcha-atrás) — o mesmo código do AgOpenGPS.

## No AgOpenWeb

- "Dual GPS" ligado.
- Orientação das antenas: `DualHeadingOffset` no AgOpenWeb **ou** `CONFIG HEADING OFFSET` no UM982 — só um dos dois.
- `IsRollInvert` / `RollZero` aplicam-se ao roll do KSXT e ao do `$PANDA`.
- O `$KSXT` não traz HDOP nem idade das correções (o AgOpenWeb mostra "—").
- `MinFixQuality` (por defeito 4 = RTK fixo): abaixo disso o AgOpenWeb marca o fix como inválido. Em bancada pôr 1 ou 2.

## Ajustes (topo de `zHandlers.ino`)

- `KSXT_MIN_HDG_QUALITY` 2 = aceita float; 3 = só RTK fixo.
- `TM171_SWAP_ROLL_PITCH`, `TM171_INVERT_ROLL`, `TM171_ROLL_OFFSET_DEG`: montagem do TM171 (só afecta o `$PANDA`). "Use Y axis" também troca roll/pitch.
- `TM171_YAW_SIGN`: 0 = sentido do yaw aprendido contra o KSXT (só com rotação real do conjunto); +1/-1 força-o.
- `GGA_HOLD_MS`: o UM982 às vezes manda GGA com posição válida mas `00` satélites / HDOP `9999.0`; o `$PANDA` repete os últimos valores bons até 2 s.
- Diagnóstico (pôr os três a 0 no trator):
  - `FUSION_DEBUG`: uma linha `[fusion] KSXT ...` ou `[fusion] PANDA ...` por segundo (qualidade do heading `q`, alinhamento do TM171, velocidade em km/h e de onde vem).
  - `RAW_NMEA_DEBUG`: cada GGA e KSXT tal como o UM982 os envia, com `[raw]`.
  - `SEND_TO_USB`: cada frase enviada ao AgOpenWeb aparece também no monitor série.

## Mensagens de estado (`zStatus.ino`)

Com `STATUS_MESSAGES 1` (ligado) o monitor série mostra linhas `[estado] ...` **só quando algo muda** (o novo estado tem de durar 1 s), por isso podem ficar ligadas no trator:

- Ethernet: cabo ligado (com o IP do módulo e o destino) / cabo desligado
- AgOpenWeb a comunicar (com o IP do PC) / sem comunicar há 5 s
- Correções RTCM a chegar do AgOpenWeb / pararam há 5 s
- GPS: primeira posição; mudanças de fix (sem fix, GPS simples, DGPS, RTK float, RTK fixo) com satélites e idade das correções; sem GGA do UM982 há 5 s
- Dual: heading das duas antenas OK (a enviar `$KSXT`) / sem heading dual ou sem KSXT (a enviar `$PANDA`)

LEDs: verde = dual (KSXT), vermelho = heading do TM171, vermelho a piscar = TM171 perdido.

Notas:
- "CRC was bad" do TM171 já não é impresso; fica contado em `TM171crcErrors` (na linha `[fusion]`).
- O ciclo de deteção do Keya lê o TM171 enquanto espera; o arranque descarta as épocas GPS acumuladas durante o `setup()`.
