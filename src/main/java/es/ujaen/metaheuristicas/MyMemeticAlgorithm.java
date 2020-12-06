/*
 * Copyright (C) 2020 agvico
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

import java.util.List;
import org.uma.jmetal.algorithm.multiobjective.mocell.MOCell;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAII;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.BinarySolution;
import org.uma.jmetal.util.archive.BoundedArchive;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.neighborhood.Neighborhood;

/**
 *
 * CLASE DE EJEMPLO PARA LA CREACION DE UN ALGORITMO MEMETICO PARA EL PROBLEMA DE EPM
 * 
 * BASADO EN EL ALGORITMO GENETICO NSGA-II (Para otro algoritmo, simplemente cambiar la clase de la que se herede)
 * 
 * SI OS FIJAIS, AL HEREDAR DE NSGA-II TODA SU FUNCIONALIDAD PERMANECE INTACTA. PARA MODIFICARLO CON NUESTRO CÓDIGO
 * PARA FORMAR UN MEMÉTICO, DEBEMOS DE SOBRECARGAR EL METODO replacement() que es la función de reemplazo.
 * 
 * 
 * En nuestro caso debemos aplicar el algoritmo de BL en la población offspring y tras esto, llamar al método super()
 * para que no se pierda el proceso de reemplazo original.
 * 
 * @author agvico
 */
public class MyMemeticAlgorithm extends MOCell<BinarySolution>{
    
    public MyMemeticAlgorithm(Problem<BinarySolution> problem, int maxEvaluations, 
            int populationSize, BoundedArchive<BinarySolution> archive, 
            Neighborhood<BinarySolution> neighborhood, CrossoverOperator<BinarySolution> crossoverOperator, 
            MutationOperator<BinarySolution> mutationOperator, SelectionOperator<List<BinarySolution>, BinarySolution> selectionOperator, 
            SolutionListEvaluator<BinarySolution> evaluator) {
        super(problem, maxEvaluations, populationSize, archive, neighborhood, crossoverOperator, mutationOperator, selectionOperator, evaluator);
    }

    @Override
    protected List<BinarySolution> replacement(List<BinarySolution> population, List<BinarySolution> offspringPopulation) {
        
        // AQUI DEBEMOS JUGAR CON offspringPopulation aplicando la BL.
        //BUSQUEDA TABU
        
        
        // Finalmente, debemos llamar SIEMPRE a super() para realizar el proceso de reemplazo original
        return super.replacement(population, offspringPopulation); //To change body of generated methods, choose Tools | Templates.
    }
    
    
    
    
}
