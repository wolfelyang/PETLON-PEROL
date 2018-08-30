import os
import numpy as np
import time, random, math
import matplotlib.pyplot as plt
#from taxi import TaxiEnv
import planner
import time

import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

import bwi_planning_common.msg
import bwi_msgs.msg
import plan_execution.msg

import time


def generateplan():
    clingopath = "clingo"
    initial = "initial.lp"
    goal = "goal.lp"
    planning = "navigation.asp navigation_facts.asp all_actions.asp"
    qvalue = "q.lp"
    return planner.compute_plan(clingopath = clingopath, initial = initial, goal=goal, planning=planning, qvalue=qvalue, printout=True)

def generateRestorePlan():
    clingopath = "clingo"
    initial = "restore_initial.lp"
    goal = "restore_goal.lp"
    planning = "navigation.asp navigation_facts.asp all_actions.asp"
    qvalue = ""
    return planner.compute_plan(clingopath = clingopath, initial = initial, goal=goal, planning=planning, qvalue=qvalue, printout=True)




def getLogicalAction(plantrace,i):
    unit = plantrace[i]
    action = unit[1]
    actionname = action[:action.find('(')]
    actionparameter = action[action.find('(')+1:action.find(')')]
    return action,actionname,actionparameter

def simulate_door_openning_time(doorname):
    if doorname in ['d3_414a1','d3_414b1']:
        return random.uniform(1,5)
    if doorname in ['d3_414a2','d3_414b2']:
        return random.uniform(5,10)
    if doorname in ['d3_414a3','d3_414b3']:
        return random.uniform(10,15)
    return random.uniform(15,20)

def executeLogicalAction(actionname,actionparameter):
    state_next_factorized = []
    try:
        print "Execute ",actionname,actionparameter
        rospy.init_node('execute_logical_goal')
        client = actionlib.SimpleActionClient('execute_logical_goal', bwi_msgs.msg.LogicalActionAction)
        client.wait_for_server()
        atom = bwi_planning_common.msg.PlannerAtom()
        atom.name = actionname
        atom.value = [actionparameter]
        goal = bwi_msgs.msg.LogicalActionGoal(atom)

        start = time.time()
        if actionname == "opendoor":
            waittime = simulate_door_openning_time(actionparameter)
            print "waiting ",waittime, "seconds for door to be opened..."
            time.sleep(waittime)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        end = time.time()
        elapsedtime = end-start
        print "elapsed time:"
        reward = elapsedtime * (-2)
        print "elapsed time",elapsedtime,"reward",reward
        if result.success:
            print "Success!"
        else:
            print "Failed because " + result.status
        for obs in result.observations:
            if obs.name[0] == '-':
                continue
            print obs.name,
            if len(obs.value) > 0:
                print "(" + ",".join(obs.value) + ")"
                state_next_factorized.append(obs.name+"("+",".join(obs.value) + ")")
            else:
                print " "
                state_next_factorized.append(obs.name)
        state_next = encode_state(state_next_factorized)
        return result, state_next, reward
    except rospy.ROSInterruptException:
        pass

def encode_state(state_factorized):
    if state_factorized in statelist:
        return statelist.index(state_factorized)
    else:
        statelist.append(state_factorized)
        return statelist.index(state_factorized)

def encode_action(action):
    if action in actionlist:
        return actionlist.index(action)
    else:
        actionlist.append(action)
        return actionlist.index(action)

def gotoInitialState():
    print "go back to initial state..."
    plantrace = []
    plantrace = generateRestorePlan()
#
    for i in xrange(len(plantrace)-1):
        action,actionname,actionparameter = getLogicalAction(plantrace,i)
        if action != None:
            result,_,_ = executeLogicalAction(actionname,actionparameter)
        elif action == None:
            print "action not found",action,i
    print "Robot initial state restored."



def generate_goal_file(planquality):
#    print "output new goal file"
    goalfile = open("goal.lp","w")
    goalfile.write("#program check(k).\n")
    goalfile.write(":- not at(l3_414a,k), query(k).\n")
    goalfile.write(":- C <= "+str(planquality)+",cost(C,k),query(k).")
    goalfile.close()

def restore_goal_file():
#    print "restore goal file"
    goalfile = open("goal.lp","w")
    goalfile.write("#program check(k).\n")
    goalfile.write(":- not at(l3_414a,k), query(k).\n")
    goalfile.close()


def pause():
    os.system('read -s -n 1 -p "Press any key to continue...\n"')

def throwdice(threshold):
    rand =  random.uniform(0,1)
    if rand < threshold:
        return True
    else:
        return False

def calculateplanquality(ro_table,stateaction):
    planquality = 0
    for (state,action) in stateaction:
        planquality += int(math.floor(ro_table[state,action]))
 #       taxirow,taxicol,_,_ = env.decode(state)
 #       actionname = getActionName(action)
 #       print "ro(pos(",taxirow,taxicol,")",actionname,")=",ro_table[state,action],int(math.floor(ro_table[state,action])),planquality
 #   print "plan quality:",planquality
#    pause()
    return planquality

def generate_rovalue_from_table(q_table_lp,ro_table):
#    print "output qvalues"
    qt = set(q_table_lp)
    qfile = open("q.lp","w")
    for (state,action) in qt:
        actionname = actionlist[action]
        symbolicstate = statelist[state]
    #    comment = "% taxi:"+str(taxirow)+","+str(taxicol)+",passenger:"+str(passidx)+"\n"
        qrule = "ro(("+','.join(str(e) for e in symbolicstate) +"),"+actionname+","+str(int(math.floor(ro_table[state,action])))+").\n"
        print qrule
        qfile.write(qrule)
    qfile.close()



if __name__=='__main__':



#    env = TaxiEnv()


    nA = 1000
    nS = 1000



    # Parameters
    LEARNING_RATE_Q = 1
    LEARNING_RATE_R = 1
    DISCOUNT = 0.001
    EPSILON = 0.01
    BETA = 0.3

    runs = 1 # multiple runs
    episodes = 20
    reward_epi = np.zeros((runs, episodes))

    # multiple runs
    for run in xrange(runs):
        q_table = np.zeros((nS, nA))
        ro_table = np.zeros((nS, nA))
        statelist = []
        actionlist = []
        q_table_lp = []

        state_factorized = ["at(l3_410)"]
        state = encode_state(state_factorized)

    #    restore_goal_file()
        converged = False
    # Episodes
        explore = True
        plantrace = []
        for episode in xrange(episodes):
            # Refresh state
            executeLogicalAction("teleport","")
            stateaction = []
            if episode % 50 == 0:
                print "Run",run,",Episode",episode
            planquality = 0
            generate_rovalue_from_table(q_table_lp,q_table)
            if explore:
                print "generate new plan..."
                oldplan = plantrace
                plantrace = generateplan()
            #    exit()
                if plantrace == None:
                    print "Run",run
                    print "No plan found at Episode",episode
                    converged = True
                    break
                    plantrace = oldplan
            if not explore:
                print "continue executing previous plan..."
            done = False
            total_reward = 0

            # Run episode
            for i in xrange(len(plantrace)-1):
                print statelist
                print actionlist
                terminate = False
                logicalaction,actionname,actionparameter = getLogicalAction(plantrace,i)
                action = encode_action(logicalaction)
                if logicalaction != None:
                    result, state_next, reward = executeLogicalAction(actionname,actionparameter)
                    if not result.success:
                        reward = -100
                        terminate = True
                    else:
                        print "next state", state_next
                        print "reward",reward
                        stateaction.append((state,action))
                #    state_next, reward, done, info = env.step(action)
                #    taxirow,taxicol,passidx,destidx = env.decode(state_next)
                elif logicalaction == None:
                    print "action not found",action,i

                q_table[state, action] += 0.1 * (reward - ro_table[state,action] + max(q_table[state_next, :]) - q_table[state, action])
                ro_table[state, action] += 0.5 * (reward + max(q_table[state_next, :]) - max(q_table[state, :])- ro_table[state,action])
                print "q value:", q_table[state, action]
                print "ro value", ro_table[state, action]
                if terminate:
                    print "execution failure occured, need to restart"
                    break


                if (state,action) not in q_table_lp:
                    q_table_lp.append((state,action))

                state = state_next
                total_reward += reward

            print "Plan execution finished. Go back to initial state"
            executeLogicalAction("teleport","")
            executeLogicalAction("closealldoors","")

            planquality = calculateplanquality(q_table,stateaction)
            reward_epi[run, episode] = total_reward
            print "run",run,"episode",episode,"total reward",total_reward,"plan quality",planquality
            eps = 0.2
            explore = throwdice(eps) and not converged
            explore=True
            if explore:
                generate_goal_file(planquality)



    # env.close()

    #plot
    #plt.figure(figsize=(15, 10))
    #plt.xlabel('Episode', fontsize=20)
    #plt.xlim(-4, episodes+4)
    #plt.ylabel('Average Reward', fontsize=20)
    #mean_reward = np.mean(reward_epi, axis=0)
    #std_reward = np.std(reward_epi, axis=0)
    #plt.errorbar(xrange(episodes), mean_reward, color='b', linewidth=1.5)
    #plt.fill_between(xrange(episodes), (mean_reward - std_reward), (mean_reward + std_reward), color='b', alpha=0.3)
    #plt.savefig('cumulative_reward_curve_symbolic.png')
    # plt.show()
