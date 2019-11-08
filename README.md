# MonitoramentoCarro
> Internet das Coisas (IoT) - Monitoramento de um Carro utilizando uma tecnologia IoT

<p>A ideia do projeto é fazer o monitoramento remoto de um carro, utiliza-se dados de sensores externos ao carro como o acelerômetro e GPS. Os dados são gravados em um cartão Micro SD e também enviados para um servidor utilizando-se as tecnologias LoRa e LoRaWAN</p>
<p>Em adição, tem-se a ideia de fazer esse controle com dados fornecidos pelo próprio carro via Rede CAN.</p>

![Carro_Isometrica](https://user-images.githubusercontent.com/51264248/68385092-cfd6a080-0137-11ea-8c6b-b8eb40368ee3.jpg)


## Instalação

Linux:
```sh
Ainda não há suporte do Mbed Studio para Linux
```

OS / Windows:

```sh
https://os.mbed.com/studio/
```

[![ARM Mbes OS](https://user-images.githubusercontent.com/51264248/68382976-86845200-0133-11ea-8274-3855aa66dab3.png)]


## Exemplo de uso

<p>O projeto pode ser utilizado das mais diversas formas, não limitando-se apenas a idéia de monitoramento de um carro.</p>
<p>No nosso exemplo, o monitoramento realizado é bem simples. Apenas obtemos dados de localização, temperatura, aceleração, tempo (data e hora) e armazenamos esses dados em um cartão Micro SD ou armazenamos em um servidor na Web.</p>
<p>Um outro exemplo de utilização, seria usar os dados do acelerômetro em conjunto com o giroscópio e tentar medir a qualidade asfáltica (pode ser um trabalho bem difífcil...).</p>
<p>Um simples controle de localização de um veículo qualquer, para descobrir sua posição em tempo real ou armazenar a rota percorrida por um veículo durante um certo intervalo de tempo (carro, bicicleta, ou até mesmo uma pessoa...).</p>
<p>Controlar através da velocidade e trepidação, se os motoristas estão passando em alta velocidade por buracos ou lombadas, podendo dessa forma verficiar se eles estão intencionalmente tentando avariar os veículos da empresa.</p>
<p>Muito mais pode ser feito, vai depender da criatividade. :)</p>

![carro](https://user-images.githubusercontent.com/51264248/68383828-28586e80-0135-11ea-812f-3dbc1efe79fc.jpg)


## Configuração para Desenvolvimento

<p>Toda a aplicação pode ser criada e programada utilizando-se o compilador online disponibilizado pela própria ARM (ARM Copyright © 2019 Arm Limited (or its affiliates) ). </p>
<p>Contudo, é recomendável utilizar o Mbed OS Studio, uma IDE Desktop disponibilizada pela ARM®. Isso pois nela é possível editar arquivos de configurações que ainda (novembro/2019) não são acessíveis no compilador online.</p>
<p>Após instalação, copie para a pasta "Mbed programs", a pasta do programa da sua aplicaçã, ou crie uma nova utilizando a própria IDE</p>

## Histórico de lançamentos

* 0.0.0
    * Criação: Primeira postagem. Código ainda incompleto em algumas partes e com comentários um pouco vazios.
* 0.0.1
    * CONSERTADO: Segunda postagem. Melhoria no código. Redução de ocorrência de possíveis erros. Melhoria nos comentários.
    * ADD: Edição de arquivos já existentes no repositório.
* 0.0.2
    * MUDANÇA: Melhoria na persistência em gravação no arquivo (Cartão Micro SD).
    * ADD: Adição da biblioteca para manipulação dos dados oriundos do GPS.

## Meta

<p>Elias Teodoro da Silva Junior - [@EliasTeodoro] - elias@ifce.edu.br </p>
<p>Felipe Moura de Castro - [@FelipeMoura] - felipecastro@ifce.edu.br </p> 
<p>Joao Bruno Costa Cruz - [@JoaoBruno] - joaobruno@ifce.edu.br </p>

<p>Distribuído sob a licença</p>
<p>Copyright (c) 2017, Arm Limited and affiliates. SPDX-License-Identifier: Apache-2.0.</p>
<p>Licença Apache-2.0</p>
<p>@Opensource - Este código-fonte pode ser utilizado, copiado, estudado, modificado e redistribuído sem restrições.</p>
<p>Esse modelo de Readme foi baseado no modelo disponível em: https://github.com/dbader/readme-template</p>
<p>Pedimos desculpas caso ainda esteja faltando citação a alguma licença. Ainda estamos melhorando o repositório. Ao final esperamos que tudo esteja certo.</p>

![LIT - IFCE](https://user-images.githubusercontent.com/51264248/68384078-9ac94e80-0135-11ea-979a-b23677465bea.png)

## Contribuindo

<p>Ainda iremos criar formas para possíveis contribuições.</p>

[MbedOS-image]: https://imgur.com/a/UdQ8Zk4
[MbedOS-url]: https://www.mbed.com/en/
