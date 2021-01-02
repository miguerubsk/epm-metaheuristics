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


import es.ujaen.metaheuristicas.evaluator.EvaluatorIndDNF;
import es.ujaen.metaheuristicas.operators.crossover.NPointCrossover;
import es.ujaen.metaheuristicas.operators.mutation.BiasedMutationDNF;
import es.ujaen.metaheuristicas.qualitymeasures.SuppDiff;
import es.ujaen.metaheuristicas.qualitymeasures.WRAccNorm;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.BinaryProblem;
import org.uma.jmetal.solution.BinarySolution;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.archive.BoundedArchive;
import org.uma.jmetal.util.archive.impl.CrowdingDistanceArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.RankingAndCrowdingDistanceComparator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;
import org.uma.jmetal.util.neighborhood.impl.C9;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import picocli.CommandLine;
import picocli.CommandLine.Parameters;
import picocli.CommandLine.Option;
import picocli.CommandLine.Command;

/**
 *
 * @author agvico
 */
@Command(name = "java -jar <file.jar>", mixinStandardHelpOptions = true, version = "1.0",
        description = "Execute the proposed memetic algorithm.")
public class Main implements Callable<Integer> {

    private final Logger logger = Logger.getLogger("Main");

    @Parameters(index = "0", description = "The training file to use.", paramLabel = "trainingFile")
    private String trainingFile = "";

    @Parameters(index = "1", description = "The test file to use.", paramLabel = "trainingFile")
    private String testFile = "";
    
      @Option(names = {"-p", "--popSize"}, description = "The number of individuals in the population", defaultValue = "100")
    private Integer populationSize = 100;

    @Option(names = {"-L", "--labels"}, description = "The number of fuzzy linguistic labels to use", defaultValue = "3")
    private Integer numLabels = 3;

    @Option(names = {"-E", "--evaluations"}, description = "The maximum number of evaluation to use", defaultValue = "25000")
    private Integer maxEvaluations = 25000;

    @Option(names = {"-c", "--crossover"}, description = "Crossover probabiltiy", defaultValue = "0.9")
    private Double crossoverProbability = 0.9;

    @Option(names = {"-m", "--mutation"}, description = "Mutation probability", defaultValue = "0.1")
    private Double mutationProbability = 0.1;

    @Option(names = {"-s", "--seed"}, description = "The seed for the random number generator", defaultValue = "1")
    private Integer seed = 1;

    @Override
    public Integer call() throws Exception {
        logger.setLevel(Level.INFO);

        // Choose the problem (in this case, an EPM Problem)
        Problema problem = (Problema) ProblemUtils.<BinarySolution>loadProblem("es.ujaen.metaheuristicas.Problema");

        // Configure the problem
        problem.readDataset(trainingFile);
        problem.setInitialisationMethod(Problema.ORIENTED_INITIALISATION);
        problem.setNumberOfLabels(numLabels);
        problem.setNumberOfObjectives(2);
        problem.setEvaluator(new EvaluatorIndDNF());
        problem.addObjective(new WRAccNorm());
        problem.addObjective(new SuppDiff());

        // Specify the seed of the problem 
        long t = System.currentTimeMillis();
        JMetalRandom.getInstance().setSeed(seed);

        // Choose the crossover algorithm: In this case, the 2-point crossover
        NPointCrossover<BinarySolution> crossover = new NPointCrossover<>(crossoverProbability, 2);

        // Mutation operator
        MutationOperator mutation = new BiasedMutationDNF(mutationProbability);

        // Selection operator: In this case, binary tournament
        BinaryTournamentSelection<BinarySolution> selection = new BinaryTournamentSelection<>(new RankingAndCrowdingDistanceComparator<>());

        // Set Dominance Comparator: WARNING! jMetal MINIMISES objectives, whereas we  MAXIMISE THEM. It is mandatory to REVERSE THIS COMPARATOR !
        DominanceComparator<BinarySolution> dominanceComparator = new DominanceComparator<>();

        // The full population (the one with patterns for all classes)
        List<BinarySolution> fullPopulation = new ArrayList<>();
        for (int clazz = 0; clazz < problem.getNumberOfClasses(); clazz++) {  // Run the algorithm for each class of the problem

            logger.info("Running class " + clazz);

            // Set the class of the population
            problem.setClass(clazz);

            // Create the algorithm
            CrowdingDistanceArchive<BinarySolution> archive = new CrowdingDistanceArchive<>(0);
            C9<BinarySolution> neighborhood = new C9<BinarySolution>(10,10);
            SequentialSolutionListEvaluator<BinarySolution> evaluator = new SequentialSolutionListEvaluator<>();
            // NOTE:  Replace this with your own ALGORITHM CONSTRUCTOR !!!!!
            MyMemeticAlgorithm algorithm = new MyMemeticAlgorithm(problem, maxEvaluations, 
                    populationSize, archive, neighborhood, (CrossoverOperator) crossover, 
                    mutation, selection, evaluator);


            // Execute the algorithm
            AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm).execute();

            // Get the results
            List<BinarySolution> nonDominatedSolutions = SolutionListUtils.getNondominatedSolutions(algorithm.getResult());
            
            // Remove repeated patterns and add them to the final result
            fullPopulation.addAll(removeRepeatedPatterns(nonDominatedSolutions, problem));

            long computingTime = algorithmRunner.getComputingTime();

            logger.info("Total execution time: " + computingTime + "ms");
        }

        // TODO: test Save results filts
        logger.info("Testing results...");
        ResultWriter writer = new ResultWriter(trainingFile + "_tra.txt", testFile, fullPopulation, problem, true);
        writer.writeTrainingMeasures();
        
        logger.info("Finished Execution.");
        return 0;
    }
    
   
    
    
    /**
     * It removes the repeated patterns in a set of patterns
     * 
     * @param patterns the set of patterns
     * @param problem  the problem definition
     * @return 
     */
    private List<BinarySolution> removeRepeatedPatterns(List<BinarySolution> patterns, BinaryProblem problem) {
        BitSet marks = new BitSet(patterns.size());
        List<BinarySolution> result = new ArrayList<>();
        
        for(int i = 0; i < patterns.size(); i++){
            if(!marks.get(i)){  // pattern i is not marked for removal
                for(int j = i + 1; j < patterns.size(); j++){
                    if(!marks.get(j)){
                        // check if all variables are equal
                        BinarySolution Pi = patterns.get(i);
                        BinarySolution Pj = patterns.get(j);
                        if(equals(Pi, Pj, problem)){
                            marks.set(j);
                        }
                    }
                }
            }
        }
        
        
        for(int i = 0; i < patterns.size(); i++){
            if(!marks.get(i)) result.add(patterns.get(i));
        }
        
        return result;
    }
    
    
    
    
    /**
     * It checks whether two patterns are equals. 
     * 
     * Two patterns are equals if their participating rules are equal.
     * 
     * @param P1
     * @param P2
     * @param problem
     * @return 
     */
    private boolean equals(BinarySolution P1, BinarySolution P2, BinaryProblem problem) {
         for(int k = 0; k < problem.getNumberOfVariables(); k++){
              if(  (P1.getVariableValue(k).cardinality() > 0 && P1.getVariableValue(k).cardinality() < problem.getNumberOfBits(k)) &&   // Both variables participates in the rule
                   (P2.getVariableValue(k).cardinality() > 0 && P2.getVariableValue(k).cardinality() < problem.getNumberOfBits(k))
                 ){
                  if(!P1.getVariableValue(k).equals(P2.getVariableValue(k))) return false;
              }
                  
         }
         return true;
    }
    
    
    
    
    
    

    public static void main(String[] args) {

        // The logic of the main() method must be implemented in the call() method.
        int exitCode = new CommandLine(new Main()).execute(args);
        System.exit(exitCode);
    }

}
