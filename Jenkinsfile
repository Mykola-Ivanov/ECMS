
pipeline {
  agent any
  stages {
    stage('Checkout') 
    {
      steps
      {
        // Change the current working directory to the specified path
        dir(path: 'D:/jenkins_nodes/default_build_node') {
          // Print the current working directory
          echo "Current working directory: ${pwd()}"
          // clear workspace
          // cleanWs()
        
          // Checkout the code from the repository
          checkout scm 

          echo "Files in the current directory: ${bat(script: 'dir', returnStdout: true)}"
          echo "Current branch: ${env.BRANCH_NAME}"
          echo "Current commit ID: ${env.GIT_COMMIT}"
          echo "Current user: ${env.USER}"
          echo "Current workspace: ${env.WORKSPACE}"
          echo "Current environment variables: ${env}"
          echo "Current Jenkins node: ${env.NODE_NAME}"
          echo "Current Jenkins job name: ${env.JOB_NAME}"
          echo "Current Jenkins build number: ${env.BUILD_NUMBER}"
          echo "Current Jenkins build URL: ${env.BUILD_URL}"
          echo "Current Jenkins workspace directory: ${env.WORKSPACE}"
          echo "Current Jenkins executor number: ${env.EXECUTOR_NUMBER}"
          echo "Current Jenkins label: ${env.NODE_LABELS}"
  
        }
      }
    }
    stage('Build')
    {
      steps 
      {
        script 
        {
          dir(path: 'D:/jenkins_nodes/default_build_node') {
            // Print the current working directory
            echo "Current working directory: ${pwd()}"
            // Execute the build command
            bat 'C:/Users/mykol/.platformio/penv/Scripts/platformio.exe run --environment upesy_wroom'
          }
        }
      }
    }
    stage('Generate Report') 
    {
      steps 
      {
        script 
        {
          dir(path: 'D:/jenkins_nodes/default_build_node') {
            // Print the current working directory
            echo "Current working directory: ${pwd()}"
            // Execute the static analysis command and save the output to 'analysis.txt' file
            def analysisResult = bat script: 'python utility-scripts/cppcheck-run-wrapper.py', returnStatus: true
            if (analysisResult != 0) {
              echo "Static analysis report generation failed with exit code ${analysisResult}"
            } else {
              echo "Static analysis report generated successfully."
            }

            // //create variable to store report generation result
            // def reportDataGenerationResult = bat(script: 'python utility-scripts/parse-static-analysis-for-nested-data.py analysis.txt output.json', returnStatus: true)
            // // Check result report data generation
            // if (reportDataGenerationResult != 0) {
            //   echo "Report data generation failed with exit code ${reportDataGenerationResult}"
            // } else {
            //   echo "Report data generated successfully."
            // }
          }
        }
      }
    }
    stage('Archive')
    {
      steps 
      {
        script 
        {
          dir(path: 'D:/jenkins_nodes/default_build_node') {
            // Archive 
            archiveArtifacts artifacts: '**/.pio/build/upesy_wroom/*, analysis.txt, output.json', allowEmptyArchive: true, fingerprint: true
          }
        }
      }
    }
  }
  post{
      always
      {
        script 
        {
          dir(path: 'D:/jenkins_nodes/default_build_node') {
            // Publish TestNG report publisher.
            testNG(showFailedBuilds: true,                              // XXX: not generated so far 
                   unstableFails: 5,                                    // TODO: add testNG report generation
                   unstableSkips: 25,
                   failedFails:  10,
                   failedSkips:   50,)
            // Publish HTML report publisher.          
            publishHTML (target : [allowMissing: false,                 // XXX: not generated so far 
                                   alwaysLinkToLastBuild: true,         // TODO: add HTML report generation
                                   keepAll: true,
                                   reportDir: 'reports',
                                   reportFiles: 'CustomReport.html',
                                   reportName: 'Custom Report Name',
                                   reportTitles: 'Custom Report Title'])
            // Publish the static analysis report
            publishCppcheck pattern:'report_cppcheck.xml'
          }
        }
      }
  }
}