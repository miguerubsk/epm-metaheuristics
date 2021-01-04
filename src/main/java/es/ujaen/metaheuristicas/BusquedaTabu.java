/*
 * Copyright (C) 2020 mamechapa
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package es.ujaen.metaheuristicas;

import es.ujaen.metaheuristicas.attributes.TablaContingencia;
import es.ujaen.metaheuristicas.evaluator.EvaluatorIndDNF;
import es.ujaen.metaheuristicas.fuzzy.FuzzySet;
import es.ujaen.metaheuristicas.qualitymeasures.ContingencyTable;
import es.ujaen.metaheuristicas.qualitymeasures.QualityMeasure;
import es.ujaen.metaheuristicas.qualitymeasures.WRAccNorm;
import java.util.Collections;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.apache.commons.collections4.list.AbstractLinkedList;
import org.uma.jmetal.solution.BinarySolution;

/**
 *
 * @author mamechapa
 */
public class BusquedaTabu {

    /**
     * The problem of the search
     */
    private Problema problem;

    /**
     * Default constructor with an associated problem
     *
     * @param problem
     */
    public BusquedaTabu(Problema problem) {
        this.problem = problem;
    }

    /**
     * Method that performs the search over the given set of fuzzy lablels.
     *
     * @param initialSolution The initial solution of the search
     * @param currentPopulation The current population of patterns
     * @param evaluator The patterns' evaluator
     * @return A new set of LLs with new fuzzy definitions.
     */
    public List<List<FuzzySet>> doBusquedaTabu(List<BinarySolution> currentPopulation) {

        /**
         * AQUI ES DONDE DEBE DE IR VUESTRO CÓDIGO RELATIVO A LA CREACIÓN DE UNA
         * BÚSQUEDA LOCAL.
         *
         */
        // Firstly, evaluate de initial population
        double initialQuality = evaluate(problem.getFuzzySets(), currentPopulation, (EvaluatorIndDNF) problem.getEvaluator(), new WRAccNorm());

        //Creamos las variables necesarias para la ejecucion del algoritmo
        Integer evaluacion = 0;
        Integer contadorReinicializacion = 0;
        Integer posicion = 0;
        double costeMejorSolucion;
        double costeSolucionActual;
        double costeSolucionParcial;

        //Creamos las estructuras necesarias para las soluciones y memorias
        List<List<FuzzySet>> solucionParcial = null;
        List<List<FuzzySet>> mejorSolucion = null;
        List<List<FuzzySet>> solucionActual = null;
//        List<List<FuzzySet>> solucion = null;
        ConcurrentLinkedQueue<Integer> listaTabu = new ConcurrentLinkedQueue<>();
        Vector<Integer> memoriaLargoPlazo = new Vector<>();

        //Generamos una solucion inicial aleatoria
        // IMPORTANTE: CLONAR INITIAL SOLUTION PARA QUE NO OCURRAN COSAS EXTRAÑAS.
        List<List<FuzzySet>> solucion = problem.getFuzzySets();

        //Usamos como mejor solucion y solucion actual la solucion generada
        mejorSolucion = solucion;
        solucionActual = mejorSolucion;

        //Inicializamos el vector de marcados para la seleccion de vecinos
        Vector<Boolean> marcados = new Vector<>();
        for (int i = 0; i < 100; i++) {
            marcados.add(Boolean.FALSE);
        }

        //Calculamos el coste de la solucion que hemos generado
        costeMejorSolucion = evaluate(solucion, currentPopulation, (EvaluatorIndDNF) problem.getEvaluator(), new WRAccNorm());

        costeSolucionActual = costeMejorSolucion;

        //Iniciamos la lista tabu y la memoria a largo plazo
        for (int i = 0; i < 20; i++) {
            listaTabu.offer(-1);
        }

        for (int i = 0; i < 100; i++) {
            memoriaLargoPlazo.add(0);
        }

        try {
            //Ejecutamos el algoritmo hasta que realicemos el numero de evaluaciones establecido
            while (evaluacion < 5000) {
                Integer numVecinos = 10;

                //Obtenemos la posicion de menor aporte
                List<FuzzySet> anterior = null;
                double costeNuevo = 0;
                double costeAnterior = 0;
                for (int i = 0; i < solucion.size(); i++) {
                    anterior = solucion.get(i);
                    solucion.remove(i);
                    costeNuevo = evaluate(solucion, currentPopulation, (EvaluatorIndDNF) problem.getEvaluator(), new WRAccNorm());
                    if (costeNuevo > costeAnterior) {
                        posicion = i;
                    }
                    solucion.add(i, anterior);
                }
                //Limpiamos el vector de marcados para realizar la seleccion de vecinos
                marcados.clear();
                for (int i = 0; i < 100; i++) {
                    marcados.add(Boolean.FALSE);
                }

                //GUardamos el elemento que vamos a sustituir
                List<FuzzySet> elementoAnterior = solucion.get(posicion);
                costeSolucionParcial = 0;

                evaluacion++;

                //Generamos y evaluamos los vecinos, obteniendo una nueva solucion con el mejor vecino
                solucionActual = evaluarVecinos(numVecinos, marcados, solucionActual, costeSolucionActual, listaTabu, solucionParcial, costeSolucionParcial, elementoAnterior, posicion, currentPopulation);

                //Calculamos el coste de esta nueva solucion
                costeSolucionActual = evaluate(solucionActual, currentPopulation, (EvaluatorIndDNF) problem.getEvaluator(), new WRAccNorm());

                //Actualizamos la lista tabu y la memoria a largo plazo
//                actualizarMemoriaLargoPlazo(memoriaLargoPlazo, solucionActual);
//                actualizarListaTabu(listaTabu, elementoAnterior);

                /*Si la solucion que hemos obtenido tiene un coste mejor 
                que nuestra mejor solucion, la solucion actual la tomamos como mejor solucion.
                Reiniciamos el contador para la reinicializacion.
                Si no mejora el coste, sumamos 1 al contador de reinicializacion.*/
                if (costeSolucionActual > costeMejorSolucion) {
                    costeMejorSolucion = costeSolucionActual;
                    mejorSolucion = solucionActual;

                    contadorReinicializacion = 0;
                } else {

                    contadorReinicializacion++;
                }

                /*Si el contandor de reinicializacion llega a 100, llevamos a cabo
                el reinicio de la busqueda, realizacion una intensificacion o diversificacion
                y obtendremos una nueva solucion desde la que continuar la busqueda.*/
//                if (contadorReinicializacion == 100) {
//
//                    solucionActual = reiniciar(memoriaLargoPlazo);
//
//                    costeSolucionActual = coste(matriz, tamañoSolucion, solucionActual);
//
//                    if (costeSolucionActual > costeMejorSolucion) {
//                        costeMejorSolucion = costeSolucionActual;
//                        mejorSolucion = solucionActual;
//                    }
//
//                    //Reiniciamos las memorias
//                    reiniciarMemorias(memoriaLargoPlazo, listaTabu);
//
//                    evaluacion++;
//                    contadorReinicializacion = 0;
//                }
            }

            //Nos guardamos la mejor solucion obtenido en nuestra solucion
            solucion = mejorSolucion;
        } catch (Exception e) {
            System.err.println("metaherísticas_pr_1.Algoritmos.BusquedaTabu(): excepcion capturada: " + e.toString() + ". Iteracion: " + evaluacion + ". Contador reinicializacion: " + contadorReinicializacion);
        }
        // Return
        return solucion;
    }

    private void limpiarMarcados(Vector<Boolean> marcados, int tamaño) {
        marcados.clear();
        for (int i = 0; i < tamaño; i++) {
            marcados.add(Boolean.FALSE);
        }
    }

    private void actualizarMemoriaLargoPlazo(Vector<Vector<Integer>> memoriaLargoPlazo, List<List<FuzzySet>> vector) {
        for (int i = 0; i < vector.size(); i++) {
            memoriaLargoPlazo.get(i).setElementAt(memoriaLargoPlazo.get(i).get(vector.get(i).size()) + 1, vector.get(i).size());
        }
    }

    private void actualizarListaTabu(ConcurrentLinkedQueue<List<FuzzySet>> listaTabu, List<FuzzySet> elementoAnterior) {
        listaTabu.offer(elementoAnterior);
        listaTabu.remove();
    }

    private List<List<FuzzySet>> evaluarVecinos(Integer numVecinos, Vector<Boolean> marcados, List<List<FuzzySet>> solucionActual,
            double costeSolucionActual, ConcurrentLinkedQueue<Integer> listaTabu,
            List<List<FuzzySet>> solucionParcial, double costeSolucionParcial, List<FuzzySet> elementoAnterior, Integer posicion, List<BinarySolution> currentPopulation) {
        double costeF = 0.0;
        int mejorVecino = 0;

        //Generamos numVecinos
        while (numVecinos > 0) {
            int vecino;
            do {
                vecino = (int) (Math.random()*10 + 1);   //Generamos un vecino hasta que encontremos 
            } while (marcados.get(vecino));                 //uno que no este marcado como seleccionado

            marcados.set(vecino, Boolean.TRUE); //Marcamos el que hemos generado

            //Si el vecino no esta en la solucion ni en la lista tabu
            if (!listaTabu.contains(vecino)) {

                //Obtenemos el coste de sustituir el elemento
                List<List<FuzzySet>> solucionAux = solucionActual;
                List<FuzzySet> listaNueva = problem.generateLinguistcLabels(0, 9, vecino);
                solucionAux.set(posicion, listaNueva);
                
                costeF = evaluate(solucionAux, currentPopulation, (EvaluatorIndDNF) problem.getEvaluator(), new WRAccNorm());
                numVecinos--;

                //Si se mejora el coste con el nuevo vecino, guardamos la nueva solucion
                if (costeSolucionParcial < costeF) {
                    mejorVecino = vecino;
                    costeSolucionParcial = costeF;
                    solucionParcial = solucionAux;
                }
            }
        }

        return solucionParcial;
    }

    /**
     * it performs the mutation operator in order to modify the given fuzzy set.
     *
     * @param f
     * @return
     */
    public FuzzySet mutate(FuzzySet f) {
        // TODO: AQUI DEBÉIS DE CREAR EL NUEVO VECINO PARA INCLUIRLO EN LA SOLUCIÓN
        return f;
    }

    /**
     * It evaluates a solution (i.e. a fuzzy sets definitions) on the current
     * population of patterns.
     *
     * It returns the average value of the selected quality measure on this
     * population of patterns.
     *
     * @param solution The fuzzy sets definitions
     * @param currentPopulation The current patterns
     * @param evaluator The evaluator of the patterns
     * @param measure The quality measure to compute
     * @return
     */
    private double evaluate(List<List<FuzzySet>> solution, List<BinarySolution> currentPopulation, EvaluatorIndDNF evaluator, QualityMeasure measure) {

        return currentPopulation.parallelStream()
                .mapToDouble((BinarySolution individual) -> {
                    evaluator.doEvaluation(individual, solution, problem.getDataset());
                    ContingencyTable table = (ContingencyTable) individual.getAttribute(ContingencyTable.class);
                    return measure.calculateValue(table);
                }).sum() / (double) currentPopulation.size();

    }

}
