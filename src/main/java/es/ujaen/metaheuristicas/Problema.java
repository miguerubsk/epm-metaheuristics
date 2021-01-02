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

import es.ujaen.metaheuristicas.attributes.Clase;
import es.ujaen.metaheuristicas.evaluator.Evaluator;
import es.ujaen.metaheuristicas.exceptions.InvalidFuzzySetException;
import es.ujaen.metaheuristicas.fuzzy.FuzzySet;
import es.ujaen.metaheuristicas.fuzzy.TriangularFuzzySet;
import es.ujaen.metaheuristicas.qualitymeasures.QualityMeasure;
import org.uma.jmetal.problem.BinaryProblem;
import org.uma.jmetal.solution.BinarySolution;
import org.uma.jmetal.solution.impl.DefaultBinarySolution;
import org.uma.jmetal.util.binarySet.BinarySet;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;

import weka.core.Instances;
import weka.core.converters.ConverterUtils.DataSource;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Class the defines an Emerging Pattern Mining using Fuzzy Sets for the
 * representation of numeric variables.
 *
 * The problem needs a dataset in arff (weka) or csv format for being able to
 * extract emerging patterns that describe the discriminative characteristics
 * between the classes of the problem.
 *
 * @author agvico
 */
public class Problema implements BinaryProblem {

    public static int RANDOM_INITIALISATION = 0;
    public static int ORIENTED_INITIALISATION = 1;

    /**
     * The dataset. It contains all the information about the instances
     */
    private Instances dataset;

    /**
     * The attribute class of the problem for the extraction of rules.
     */
    private Clase<BinarySolution> clase;
    private int clas;

    /**
     * The number of objectives to be used in a MOEA algorithm. By default we
     * use only 2 objectives
     */
    private int numObjectives = 2;

    /**
     * The number of linguistic labels to be used in numeric variables. By
     * default We use 3 linguistic labels for defining a numeric variable.
     *
     * It is recomended to set this value as an odd number, such as 3,5,7,9...
     */
    private int numLabels = 3;

    /**
     * The initialisation method used
     */
    private int initialisationMethod;

    /**
     * The fuzzy sets that define the linguistic labels for each numeric
     * variable.
     *
     * If a variable is non-numeric, its row must be {@code null}
     */
    private List<List<FuzzySet>> fuzzySets;

    /**
     * The evaluator used for measuring the objectives of the individuals
     */
    private Evaluator evaluator;

    /**
     * The random number generator (It belongs to the jMetal framework)
     */
    public JMetalRandom rand;

    /**
     * It returns the total length of the chromosome.
     */
    private int length;

    /**
     * It reads an ARFF or CSV file using the WEKA API.
     *
     *
     * @param path The path of the dataset
     */
    public void readDataset(String path) {

        // First, read the dataset and select the class
        DataSource source;
        rand = JMetalRandom.getInstance();
        clase = new Clase<>();

        try {
            source = new DataSource(path);
            dataset = source.getDataSet();

            // Con esto se le fija como clase el ultimo atributo si no estuviera especificado
            if (dataset.classIndex() == -1) {
                dataset.setClassIndex(dataset.numAttributes() - 1);
            }

            // Set the number of linguistic labels and calculates its defintions
            setNumberOfLabels(numLabels);

            // calculate the length of the chromosomes
            length = 0;
            for (int index = 0; index < dataset.numAttributes(); index++) {
                if (index != dataset.classIndex()) {
                    if (dataset.attribute(index).isNumeric()) {
                        length += numLabels;
                    } else if (dataset.attribute(index).isNominal()) {
                        length += dataset.attribute(index).numValues();
                    }
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    @Override
    public int getNumberOfVariables() {
        return dataset.numAttributes() - 1;
    }

    @Override
    public int getNumberOfObjectives() {
        return numObjectives;
    }

    @Override
    public int getNumberOfConstraints() {
        return 0;
    }

    @Override
    public String getName() {
        return dataset.relationName();
    }

    @Override
    public void evaluate(BinarySolution solution) {
        // Evaluates the chromosome against the dataset using the fuzzy sets
        // definitions (if necessary). It also sets the objective values.
        evaluator.doEvaluation(solution, fuzzySets, dataset);

    }

    /**
     * It sets the number of objectives in this multi-objective problem.
     *
     * @param numObjectives
     */
    public void setNumberOfObjectives(int numObjectives) {
        this.numObjectives = numObjectives;
    }

    /**
     * It returns the number of Linguistic Labels employed in this problem for
     * representing numeric variables.
     *
     * @return
     */
    public int getNumberOfLabels() {
        return numLabels;
    }

    /**
     * It sets the number of linguistic labels employed in this problem. In
     * addition, it recalculates the Fuzzy sets definitions with the new value
     * employed.
     *
     * @param numLabels
     */
    public void setNumberOfLabels(int numLabels) {
        this.numLabels = numLabels;
        // Next, set the fuzzy linguistic labels for numeric variables
        fuzzySets = new ArrayList<>();
        for (int i = 0; i < dataset.numAttributes(); i++) {
            if (i != dataset.classIndex() && dataset.attribute(i).isNumeric()) {
                double max = getMax(i);
                double min = getMin(i);
                fuzzySets.add(generateLinguistcLabels(min, max));
            } else {
                fuzzySets.add(null);
            }
        }
    }

    public int getInitialisationMethod() {
        return initialisationMethod;
    }

    public void setInitialisationMethod(int initialisationMethod) {
        this.initialisationMethod = initialisationMethod;
    }

    public List<List<FuzzySet>> getFuzzySets() {
        return fuzzySets;
    }

    public void setFuzzySets(List<List<FuzzySet>> fuzzySets) {
        this.fuzzySets = fuzzySets;
    }

    public void addObjective(QualityMeasure measure){
        if(evaluator.getObjectives().size() < numObjectives){
            evaluator.getObjectives().add(measure);
        }
    }
    
    /**
     * Gets the evaluator used for evaluating the individuals
     *
     * @return
     */
    public Evaluator getEvaluator() {
        return evaluator;
    }

    /**
     * It sets the evaluator employed for evaluating the individuals
     *
     * @param evaluator
     */
    public void setEvaluator(Evaluator evaluator) {
        this.evaluator = evaluator;
    }

    /**
     * Get the dataset of this problem
     *
     * @return
     */
    public Instances getDataset() {
        return dataset;
    }

    /**
     * Set the dataset of this problem
     *
     * @param dataset A weka-based dataset
     */
    public void setDataset(Instances dataset) {
        this.dataset = dataset;
    }

    /**
     * Get the number of classes of the given problem.
     * @return 
     */
    public int getNumberOfClasses(){
        return dataset.numClasses();
    }
    
    /**
     * It creates a new chromosome for this problem.
     *
     * @return
     */
    @Override
    public BinarySolution createSolution() {
        // Create a random individual
        DefaultBinarySolution sol = null;
        if (initialisationMethod == RANDOM_INITIALISATION) {
            // By default, individuals are initialised at random
            sol = new DefaultBinarySolution(this);
        } else if (initialisationMethod == ORIENTED_INITIALISATION) {
            // Oriented initialisation
            sol = OrientedInitialisation(rand, 0.25);

        }

        // Set the class of this chromosome
        clase.setAttribute(sol, clas);
        return sol;
    }

    /**
     * Set the class of a pattern. All patterns share the same class.
     *
     * @param clas
     */
    public void setClass(int clas) {
        this.clas = clas;
    }

    /**
     * Get the number of bits of a given variable. The number of bits represents
     * the number of possible values for this variable.
     *
     * @param index
     * @return
     */
    @Override
    public int getNumberOfBits(int index) {
        if (dataset.attribute(index).isNumeric()) {
            return numLabels;
        } else if (dataset.attribute(index).isNominal()) {
            return dataset.attribute(index).numValues();
        } else {
            return -1; // Cambiar por excepción o algo así
        }
    }

    /**
     * It returns the length of the whole chromosome
     *
     * @return
     */
    @Override
    public int getTotalNumberOfBits() {
        return length;
    }

    /**
     * Get the maximum value of a numeric variable
     *
     * @param var
     */
    private double getMax(int var) {
        double max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < dataset.numInstances(); i++) {
            if (dataset.get(i).value(var) > max) {
                max = dataset.get(i).value(var);
            }
        }

        return max;
    }

    /**
     * Get the minimum value of a numeric variable
     *
     * @param var
     */
    private double getMin(int var) {
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.numInstances(); i++) {
            if (dataset.get(i).value(var) < min) {
                min = dataset.get(i).value(var);
            }
        }

        return min;
    }
    
    /**
     * Get the maximum value of a numeric variable
     *
     * @param var
     */
    public double getMaxP(int var) {
        double max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < dataset.numInstances(); i++) {
            if (dataset.get(i).value(var) > max) {
                max = dataset.get(i).value(var);
            }
        }

        return max;
    }

    /**
     * Get the minimum value of a numeric variable
     *
     * @param var
     */
    public double getMinP(int var) {
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.numInstances(); i++) {
            if (dataset.get(i).value(var) < min) {
                min = dataset.get(i).value(var);
            }
        }

        return min;
    }

    /**
     * It generates the triangular liguistic labels for covering from min to max
     * using the specified number of linguistic labels
     *
     * @param min
     * @return
     */
    private List<FuzzySet> generateLinguistcLabels(double min, double max) {
        double marca = (max - min) / ((double) (numLabels - 1));
        double cutPoint = min + marca / 2;
        ArrayList<FuzzySet> sets = new ArrayList<>();

        for (int label = 0; label < numLabels; label++) {
            ArrayList<Double> definitions = new ArrayList<>();
            double value = min + marca * (label - 1);

            // Creation of x0 point
            if (label == 0) {
                definitions.add(-1 * Double.MAX_VALUE);
            } else {
                definitions.add(Round(value, max));
            }

            // Creation of x1 point
            value = min + marca * label;
            definitions.add(Round(value, max));

            // Creation of x2 point
            value = min + marca * (label + 1);
            if (label == numLabels - 1) {
                definitions.add(Double.MAX_VALUE);
            } else {
                definitions.add(Round(value, max));
            }

            // Create de triangular fuzzy set
            TriangularFuzzySet set = null;
            try {
                set = new TriangularFuzzySet(definitions, 1.0);
            } catch (InvalidFuzzySetException ex) {
                Logger.getLogger(Problema.class.getName()).log(Level.SEVERE, null, ex);
            }
            sets.add(set);

            cutPoint += marca;
        }

        return sets;
    }

    /**
     * It generates the triangular liguistic labels for covering from min to max
     * using the specified number of linguistic labels
     *
     * @param min
     * @return
     */
    public List<FuzzySet> generateLinguistcLabels(double min, double max, int numLabels) {
        double marca = (max - min) / ((double) (numLabels - 1));
        double cutPoint = min + marca / 2;
        ArrayList<FuzzySet> sets = new ArrayList<>();

        for (int label = 0; label < numLabels; label++) {
            ArrayList<Double> definitions = new ArrayList<>();
            double value = min + marca * (label - 1);

            // Creation of x0 point
            if (label == 0) {
                definitions.add(-1 * Double.MAX_VALUE);
            } else {
                definitions.add(Round(value, max));
            }

            // Creation of x1 point
            value = min + marca * label;
            definitions.add(Round(value, max));

            // Creation of x2 point
            value = min + marca * (label + 1);
            if (label == numLabels - 1) {
                definitions.add(Double.MAX_VALUE);
            } else {
                definitions.add(Round(value, max));
            }

            // Create de triangular fuzzy set
            TriangularFuzzySet set = null;
            try {
                set = new TriangularFuzzySet(definitions, 1.0);
            } catch (InvalidFuzzySetException ex) {
                Logger.getLogger(Problema.class.getName()).log(Level.SEVERE, null, ex);
            }
            sets.add(set);

            cutPoint += marca;
        }

        return sets;
    }
    
    /**
     * <p>
     * Rounds the generated value for the semantics when necesary
     * </p>
     *
     * @param val The value to round
     * @param tope
     * @return
     */
    public double Round(double val, double tope) {
        if (val > -0.0001 && val < 0.0001) {
            return (0);
        }
        if (val > tope - 0.0001 && val < tope + 0.0001) {
            return (tope);
        }
        return (val);
    }

    /**
     * It generates a random indivual initialising a percentage of its variables
     * at random.
     *
     * @param rand
     * @return
     */
    public DefaultBinarySolution OrientedInitialisation(JMetalRandom rand, double pctVariables) {
        DefaultBinarySolution sol = new DefaultBinarySolution(this);
        long maxVariablesToInitialise = Math.round(pctVariables * getNumberOfVariables());
        int varsToInit = rand.nextInt(0, (int) maxVariablesToInitialise) + 1;

        BitSet initialised = new BitSet(getNumberOfVariables());
        initialised.set(dataset.classIndex());
        int varInitialised = 0;

        while (varInitialised != varsToInit) {
            int var = rand.nextInt(0, getNumberOfVariables());
            if (!initialised.get(var)) {
                BinarySet value = new BinarySet(sol.getNumberOfBits(var));
                for (int i = 0; i < sol.getNumberOfBits(var); i++) {
                    if (rand.nextDouble(0.0, 1.0) <= 0.5) {
                        value.set(i);
                    } else {
                        value.clear(i);
                    }
                }
                // check if the generated variable is empty and fix it if necessary
                if (value.cardinality() == 0) {
                    value.set(rand.nextInt(0, sol.getNumberOfBits(var)));
                } else if (value.cardinality() == sol.getNumberOfBits(var)) {
                    value.clear(rand.nextInt(0, sol.getNumberOfBits(var)));
                }
                sol.setVariableValue(var, value);
                varInitialised++;
                initialised.set(var);
            }
        }

        // clear the non-initialised variables
        for (int i = 0; i < sol.getNumberOfVariables(); i++) {
            if (!initialised.get(i)) {
                sol.getVariableValue(i).clear();
            }
        }

        return sol;
    }
    
    
    public Instances getData() {return this.dataset; }
    
}
