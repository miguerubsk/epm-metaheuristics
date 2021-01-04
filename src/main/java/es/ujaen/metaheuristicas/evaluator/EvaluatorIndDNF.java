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
package es.ujaen.metaheuristicas.evaluator;

import es.ujaen.metaheuristicas.evaluator.Evaluator;
import es.ujaen.metaheuristicas.fuzzy.FuzzySet;
import es.ujaen.metaheuristicas.attributes.Clase;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.impl.DefaultBinarySolution;
import es.ujaen.metaheuristicas.qualitymeasures.ContingencyTable;
import es.ujaen.metaheuristicas.qualitymeasures.QualityMeasure;
import weka.core.Instances;

import java.util.ArrayList;
import java.util.List;


public class EvaluatorIndDNF extends Evaluator {

    public EvaluatorIndDNF(){
        super();
        super.setObjectives(new ArrayList<>());
    }
    
    @Override
    public void doEvaluation(Solution individual, List<List<FuzzySet>> fuzzySet, Instances dataset) {
        if(individual instanceof DefaultBinarySolution) {
            DefaultBinarySolution ind = (DefaultBinarySolution) individual;
            int tp = 0;
            int fp = 0;
            int tn = 0;
            int fn = 0;

            // Now, for each instance in the dataset, calculate the coverage or not of the example

            if (! isEmpty(ind)) { // The pattern is empty or it is not valid (as the class variable contains more than one class.)
                for (int i = 0; i < dataset.numInstances(); i++) {
                    double fuzzyTrigger = 1.0;
                    int index = 0;
                    for (int var = 0; var < dataset.numAttributes() && fuzzyTrigger > 0.0; var++) {
                        if (var != dataset.classIndex()) {
                            if (participates(ind, var)) {
                                // The variable participates in the rule (all values are different from zero or one)
                                if (dataset.attribute(var).isNominal()) {
                                    // Variable nominal
                                    Double value = dataset.instance(i).value(var);
                                    if (!ind.getVariableValue(var).get(value.intValue()) && !dataset.instance(i).isMissing(var)) {
                                        // Variable (and the whole rule) does not cover the example
                                        fuzzyTrigger = 0.0;
                                    }
                                } else if (dataset.attribute(var).isNumeric()) {
                                    // Numeric variable, fuzzy computation.
                                    if (!dataset.instance(i).isMissing(var)) {
                                        double belonging = 0.0;
                                        double aux;
                                        for (int k = 0; k < ind.getNumberOfBits(var); k++) {
                                            if (ind.getVariableValue(var).get(k)) {
                                                Double value = dataset.instance(i).value(var);
                                                aux = fuzzySet.get(var).get(k).getBelongingDegree(value);
                                            } else {
                                                aux = 0.0;
                                            }
                                            belonging = Math.max(belonging, aux);
                                        }
                                        fuzzyTrigger = Math.min(belonging, fuzzyTrigger);
                                    }
                                }
                            }
                        }
                    }

                    // Get the class of the examples
                    Double classAttr = dataset.get(i).classValue();

                    // Get the attribute class of the individual
                    int clas = (int) ind.getAttribute(new Clase<DefaultBinarySolution>().getAttributeIdentifier());

                    // Fuzzy belonging degree is now calculated for the given instance. Calculate the measures
                    if (fuzzyTrigger > 0) {
                        if (clas == classAttr.intValue()) {
                            tp++;
                        } else {
                            fp++;
                        }
                    } else {
                        if (clas == classAttr.intValue()) {
                            fn++;
                        } else {
                            tn++;
                        }
                    }

                }

                // now, all individuals are evaluated so the contingency table can be created for calculating the objectives.

                ContingencyTable table = new ContingencyTable(tp, fp, tn, fn);
                ind.setAttribute(ContingencyTable.class, table);
                individual.setAttribute(ContingencyTable.class, table);
                ArrayList<QualityMeasure> measures = super.calculateMeasures(table);
                for (int i = 0; i < measures.size(); i++) {
                    ind.setObjective(i, measures.get(i).getValue());
                }
            } else {
                for (int i = 0; i < ind.getNumberOfObjectives(); i++) {
                    ind.setObjective(i, Double.NEGATIVE_INFINITY);
                }
            }
        }
    }

    @Override
    public boolean isEmpty(Solution individual) {
        if(individual instanceof DefaultBinarySolution){
            DefaultBinarySolution ind = (DefaultBinarySolution) individual;
            for(int i = 0; i < ind.getNumberOfVariables(); i++){
                if(ind.getVariableValue(i).cardinality() > 0 && ind.getVariableValue(i).cardinality() < ind.getNumberOfBits(i)){
                    // variable participates in the rules, is not empty
                    return false;
                }
            }

            return true;
        } else {
            return true;
        }
    }

    @Override
    public boolean participates(Solution individual, int var) {
        if(individual instanceof DefaultBinarySolution) {
            DefaultBinarySolution ind = (DefaultBinarySolution) individual;
            // a variable does not participate in the rule if all its values are 0 or 1.
            return ind.getVariableValue(var).cardinality() > 0 && ind.getVariableValue(var).cardinality() < ind.getNumberOfBits(var);
        } else {
            return false;
        }
    }


}
