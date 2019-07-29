pipeline {
    agent { docker image: 'bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true }

    environment {
        HOME = "$WORKSPACE"
    }

    stages {
        stage('Build') {
            stages {
                stage('Install dependencies') {
                    steps {
                        sh '''#!/bin/bash
                            ln -s `realpath dynamic_stack_decider` /catkin_ws/src/
                            ln -s `realpath dynamic_stack_decider_visualization` /catkin_ws/src/
                            cd /catkin_ws

                            rosdep update
                            rosdep install -ya
                        '''
                    }
                }

                stage('Build package') {
                    steps {
                        sh '''#!/bin/bash
                            source /opt/ros/melodic/setup.bash

                            cd /catkin_ws
                            catkin build
                        '''
                    }
                }
            }
        }

        stage('Test') {
            steps {
                sh '''#!/bin/bash
                    source /catkin_ws/devel/setup.bash
                    roscd dynamic_stack_decider

                    python-coverage run tests/test_parser.py
                    cp report.xml $HOME/report.xml
                    python-coverage html -d $HOME/coverage/
                '''
            }
        }

    }

    post {
        always {
            publishHTML([
                allowMissing: false,
                alwaysLinkToLastBuild: false,
                keepAll: false,
                reportDir: 'coverage',
                reportFiles: 'index.html',
                reportName: 'Coverage Report',
                reportTitles: ''])

            junit 'report.xml'
        }
    }
}
