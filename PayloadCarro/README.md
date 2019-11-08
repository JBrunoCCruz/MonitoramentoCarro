  Tentei explicar de uma maneira simples e com exemplos, como funciona e de que maneira se pode utilizar essa biblioteca.
  O Payload Carro foi baseado no seguinte modelo de payload: [CayenneLPP](https://github.com/myDevicesIoT/cayenne-docs/blob/master/docs/LORA.md)
 
 
  <p>Essa breve explicacao tenta esclarecer como funciona a codificacao
  e a decodificacao dos valores a serem enviados.
  Normalmente os valores com os quais trabalhamoss são numeros reais,
  ou seja, negativos, positivos e zero.
  A funcao de "envia" da biblioteca 'lora' ('send'), trabalha recebendo
  inteiros sem sinais (unsigned int) para enviar os dados pela rede.
  Dessa forma, e necessario encontrar uma maneira de enviar valores
  negativos; a solucao encontrada foi a seguinte:
  Divide-se o numero pelo seu modulo para encontrar o fator somador
  que será -1 se o numero for negativo, e 1 caso seja positivo. Apos
  isso, soma-se com 46; se o numero for negativo, o resultado da soma
  sera 45 que equivale ao sinal '-' em caractere, e entao enviamos pela
  rede um bit para informar o sinal antes de enviar o valor</p>
 
  <p>Ja que pode-se apenas enviar bytes pela rede, os numero sao divididos
  em duas partes: 1 byte para a parte inteira e 1 byte para a parte 
  fracionaria. Em seguida envia-se pela rede e o valor é reconstruido
  do outro lado.</p>
 

|Exemplo|-2.5          | | |
|:-:|:----------------------------:|:------------------:|:--------------------:|
|1  |-2/5 / abs(-2.5) = -1        |(fator somador)                            ||
|2  |46 + (-1) = 45               |(numero é negativo)                        ||
|3  |parte inteira|abs (-2.5) = 2                                             ||
|4  |parte fracionaria            |(abs (-2.5) - 2 ) * 100 = 50               ||
|5  |-|2|50|
|6  |byte 0|byte 1|byte 2|
|7  |sinal|inteira|fracionaria|
