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
import java.util.List;
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
     * @param problem 
     */
    public BusquedaTabu(Problema problem){
        this.problem = problem;
    }
    
    
    
    /**
     * Method that performs the search over the given set of fuzzy lablels.
     * 
     * @param initialSolution       The initial solution of the search
     * @param currentPopulation     The current population of patterns
     * @param evaluator             The patterns' evaluator
     * @return     A new set of LLs with new fuzzy definitions.
     */
    public List<List<FuzzySet>> doBusquedaTabu(List<List<FuzzySet>> initialSolution, List<BinarySolution> currentPopulation, EvaluatorIndDNF evaluator) {
        
                
        /**
         * AQUI ES DONDE DEBE DE IR VUESTRO CÓDIGO RELATIVO A LA CREACIÓN DE UNA BÚSQUEDA LOCAL.
         * 
         */
        // Firstly, evaluate de initial population
        double initialQuality = evaluate(initialSolution, currentPopulation, evaluator, new WRAccNorm());
        
        // IMPORTANTE: CLONAR INITIAL SOLUTION PARA QUE NO OCURRAN COSAS EXTRAÑAS.
        //List<List<FuzzySet>> currentSolution = initialSolution.copy()
        
        
        // Return
        return initialSolution;
    }
 
    
    /**
     * it performs the mutation operator in order to modify the given fuzzy set.
     * 
     * @param f
     * @return 
     */
    public FuzzySet mutate(FuzzySet f){
        // TODO: AQUI DEBÉIS DE CREAR EL NUEVO VECINO PARA INCLUIRLO EN LA SOLUCIÓN
        return f;
    }
    
    
    /**
     * It evaluates a solution (i.e. a fuzzy sets definitions) on the current population of patterns.
     * 
     * It returns the average value of the selected quality measure on this population of patterns.
     * 
     * @param solution              The fuzzy sets definitions
     * @param currentPopulation     The current patterns
     * @param evaluator             The evaluator of the patterns
     * @param measure               The quality measure to compute
     * @return 
     */
    private double evaluate(List<List<FuzzySet>> solution, List<BinarySolution> currentPopulation, EvaluatorIndDNF evaluator, QualityMeasure measure) {
        
        return currentPopulation.parallelStream()
                .mapToDouble((BinarySolution individual) -> {
                    evaluator.doEvaluation(individual, solution, problem.getDataset());
                    ContingencyTable table = (ContingencyTable) individual.getAttribute(TablaContingencia.class);
                    return measure.calculateValue(table);
                }).sum() / (double) currentPopulation.size();
                
    }
    
}
