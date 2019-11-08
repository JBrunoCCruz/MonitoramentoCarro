A biblioteca utilizada no nosso projeto, foi obtida no seguinte endereço virtual:
[MPU6050 - ARK4579](https://www.instructables.com/id/STM32F103-MPU6050-using-Mbedh/)

Nesse link, também é possível ver um breve tutorial de utilização.

<p>Nessa MPU, há a possibilidade de se utilizar 2 endereços I²C: 0x68 e 0x69.</p>
<p>Eles podem ser setados de acordo o nível lógico do pino INT. 0 -> 0x68 / 1 -> 0x69.</p>
<p>Isso possibilita o uso de 2 módulos em um mesmo projeto</p>

## Links

<p>O seguinte link é um exemplo de utilização dessa MPU no arduino, disponibilizado pelo: Filipe Flop</p>
<p>Pode ser interessante para um primeiro contato com a MPU. Aliás, nela há 3 sensores: Acelerômetro, Giroscópio e um Sensor de Temperatura</p>

[MPU6050 - Filipe Flop](https://www.filipeflop.com/blog/tutorial-acelerometro-mpu6050-arduino/)
