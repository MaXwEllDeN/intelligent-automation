## Atividade 4: Sintonia de controladores e estimação de parâmetros

1. Resolva o problema apresentado no script myNR.m utilizado o algoritmo genético descrito nas páginas 8 e 9 (speedyGA.zip) do capítulo 1 do livro "An Introduction to Genetic Algorithms" de Melanie Mitchell.

2. Implemente o ajuste dos ganhos da lei de controle PID (com filtro derivativo) "go to goal" (point stabilization) usando os critérios ISE, ITSE, IAE e ITAE como indicadores de desempenho e o algoritmo genético descrito nas páginas 8 e 9 (speedyGA.zip) do capítulo 1 do livro "An Introduction to Genetic Algorithms" de Melanie Mitchell. Na sua implementação inclua restrições semelhantes aquelas consideradas no script "gotogoalPID.m".

3. Obtenha versões discretizadas dos controladores sintonizados (ZOH e Tustin), implemente-as em linguagem LUA e teste-as usando a cena "ControleUniciclo.ttt".

4. Escolha um motor de indução no site da WEG (weg.net -- motor de indução trifásico: uso geral, baixa tensão e alto torque) obtenha os dados das curvas de desempenho e utilize a formulação apresentada no artigo intitulado "Characterization of induction machines with a genetic algorithm" para estimar os parâmetros do circuito equivalente da máquina escolhida. Caso seja necessário digitalizar os dados use, por exemplo, https://apps.automeris.io/wpd/. Veja também a formulação multi-objetivo descrita no artigo "Multi-objective identification of induction machine models".