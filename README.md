Dentro do projeto de exemplo:

	- idf.py build -> Cria um build de execução do projeto

	- idf.py -p PORT flash -> Grava no esp32 o build criado
		- PORT é -> /dev/ttyUSB0 [PORTA USB MAIS ABAIXO]
		- Pode ser necessário executar o comando: sudo chmod 666 /dev/ttyUSB0 -> para ceder permissões 

	- idf.py -p /dev/ttyUSB0 monitor -> para exibir os logs no console
		- ctrl + ] -> sair dos logs

	- Próximos passos:
		- Criar um .sh executavel que realize os comandos [ BUILDAR, GRAVAR, ABRIR MONITOR] automaticamente