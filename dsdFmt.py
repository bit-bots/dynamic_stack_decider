#!/usr/bin/env python3
'''
This file which can be used to format the dsd file is inspired by generate_specs.py.
^_^

To use this script, Plese specify the arguments in the following order:
./dsdFmt.py argv[1] argv[2] argv[3]
argv[1]: input file i.e. non-formatting file
argv[2]: output file i.e. formatted file
argv[3]: dir which contains decisions/ and actions/ directories

'''

import sys
import re
import os

space = " "
tab = space * 4 # Assume tab means 4 space as recommended for Python Style

def writeContent(writer, content):
    writer.write(content)

def stackClean(_decisions_out, _decisions_stack, _indents_dict, _result_found_dict):
    if _result_found_dict[_decisions_stack[-1]] == len(_decisions_out[_decisions_stack[-1]]):
        _indents_dict.pop(_decisions_stack[-1])
        _result_found_dict.pop(_decisions_stack[-1])
        _decisions_stack.pop()
        if _decisions_stack:
            stackClean(_decisions_out, _decisions_stack, _indents_dict, _result_found_dict)
    else:
        pass

try:
    inFile = sys.argv[1]
    print('Non-formatting dsd file is {}'.format(inFile))
    outFile = sys.argv[2]
    print('Going to write input file {}'.format(outFile))
    daDir = sys.argv[3]
    daCheck = re.search(r"/$", daDir)
    if not daCheck:
        daDir = daDir + "/"
    print('Decision elements and Action elements are stored in {}'.format(daDir))
    print('-NOW START FORMATTING')
except:
    raise AssertionError("Please specify the file input or output")

decisions_out = dict()
print('--NOW START REGISTER RETURN VAR OF DECISION ELEMENTS')
for r, _, dfs in os.walk(os.path.join(daDir, "decisions/")):
    for df in dfs:
        print('Now in file {}'.format(df))
        cn = "" # cn is the name of the class i.e. className --> cn
        with open(os.path.join(r, df), "r") as dp:
            for line in dp:
                cnResult = re.search(r"(?<=class\s)[a-zA-Z0-9]*", line)
                # I think you will not define more than 1 classes in 1 line
                if cnResult:
                    cn = cnResult.group()
                    print('found DECISION: {}'.format(cn))
                    decisions_out[cn] = list()
                rtResult = re.search(r"return", line)
                if rtResult:
                    rtvar = re.findall(r"\'[a-zA-Z0-9\_]*\'", line)
                    for var in rtvar:
                        var = var.strip("'")
                        print('found return obj: {}'.format(var))
                        decisions_out[cn].append(var)
actions_name = list()
for r, _, afs in os.walk(os.path.join(daDir, "actions/")):
    for af in afs:
        print('Now in file {}'.format(af))
        cn = ""  # cn is the name of the class i.e. className --> cn
        with open(os.path.join(r, af), "r") as dp:
            for line in dp:
                cnResult = re.search(r"(?<=class\s)[a-zA-Z0-9]*", line)
                # I think you will not define more than 1 classes in 1 line
                if cnResult:
                    cn = cnResult.group()
                    print('found ACTION: {}'.format(cn))
                    actions_name.append(cn)

with open(outFile, 'w') as outdsd:
    with open(inFile, 'r') as indsd:
        next_is_start = False
        next_is_comment = False
        comment = False
        lnr = 0
        
        # params used to specify the indents before the line_content
        decisions_stack = list()
        decision_name = ""
        indents_dict = dict()
        result_found_dict = dict()
        
        for line in indsd:
            lnr += 1
            print("NOW in line {}".format(lnr))
            comment = next_is_comment
            # I have not verified the effect of the following codes since there are no files with comments written ^_^
            line = re.sub(r'//\*\*.*?\*\*//', '', line)  # Block comments starting and ending in the same line

            if '**//' in line:
                # Block comments ending in this line
                # This line as well as the following will contain valid code
                next_is_comment = False
                comment = False
                line = re.sub(r'.*\*\*//', '', line)
            if '//**' in line:
                # Block comments starting in this line
                # This line may contain valid code, the next ones won't
                next_is_comment = True
                line = re.sub(r'//\*\*.*', '', line)

            line = re.sub(r'//.*', '', line)  # Line comments
            
            line = line.rstrip()
            if not line:
                continue

            if not comment:
                line_content = line.lstrip()
                
                if line_content.startswith('-->'):
                    # '-->' indicates the start of the whole tree
                    writeContent(outdsd, '\n' + line_content + '\n')
                    print("NOW find the start of the whole tree : {}".format(line_content.lstrip('-->')))
                    continue

                if line_content.startswith('#'):
                    # '#' indicates the start of a new subtree, so clear the params
                    decisions_stack = list()
                    indents_dict = dict()
                    result_found_dict = dict()
                    decision_name = ""
                    writeContent(outdsd, '\n' + line_content + '\n')
                    print("NOW start a new stack")
                    continue
                
                if line_content.startswith('$'):
                    # '$' indicatest the root element of the subtree
                    decision_name = line_content.lstrip('$')
                    decisions_stack.append(decision_name)
                    indents_dict[decisions_stack[-1]] = 1
                    result_found_dict[decisions_stack[-1]] = 0
                    writeContent(outdsd, line_content + '\n')
                    print("DECISION_NAME is {}".format(decision_name))
                    print("DECISIONS_STACK is {}".format(decisions_stack))
                    print("INDENTS_DICT is {}".format(indents_dict))
                    print("RESULT_FOUND_DICT is {}".format(result_found_dict))
                    continue
                
                if re.search(r'\s*-?->\s*', line_content):
                    # Arrow in line, split in decision result and call
                    result, call = re.split(r'\s*-?->\s*', line_content, 1)
                    print("TOP of the stack is {}".format(decisions_stack[-1]))
                    print("RESULT is : {}".format(result))
                    print("CALL is : {}".format(call))
                    if result in decisions_out[decisions_stack[-1]]:
                        writeContent(outdsd, tab*indents_dict[decisions_stack[-1]] + line_content + '\n')
                        result_found_dict[decisions_stack[-1]] += 1
                        print("RESULT_FOUND_DCIT is {}, after {} is found".format(result_found_dict, result))
                    else:
                        raise ValueError("Cannot find the result {} in line {}!".format(result, lnr))
                    
                    if re.search(r'\$', call):
                        # Indicates the root node of new substree
                        decision_name = call.lstrip('$')
                        decisions_stack.append(decision_name)
                        indents_dict[decisions_stack[-1]] = indents_dict[decisions_stack[-2]] + 1
                        result_found_dict[decisions_stack[-1]] = 0
                        print("DECISION_NAME is {}".format(decision_name))
                        print("DECISIONS_STACK is {}".format(decisions_stack))
                        print("INDENTS_DICT is {}".format(indents_dict))
                        print("RESULT_FOUND_DICT is {}".format(result_found_dict))
                        continue
                    
                    stackClean(decisions_out, decisions_stack, indents_dict, result_found_dict) 
