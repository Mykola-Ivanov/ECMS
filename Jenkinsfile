
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
        
          // Print the list of files in the current directory
          echo "Files in the current directory: ${bat(script: 'dir', returnStdout: true)}"
          // Print the current branch name
          echo "Current branch: ${env.BRANCH_NAME}"
          // Print the current commit ID
          echo "Current commit ID: ${env.GIT_COMMIT}"
          // Print the current user
          echo "Current user: ${env.USER}"
          // Print the current workspace
          echo "Current workspace: ${env.WORKSPACE}"
          // Print the current environment variables
          echo "Current environment variables: ${env}"
          // Print the current Jenkins node
          echo "Current Jenkins node: ${env.NODE_NAME}"
          // Print the current Jenkins job name
          echo "Current Jenkins job name: ${env.JOB_NAME}"
          // Print the current Jenkins build number
          echo "Current Jenkins build number: ${env.BUILD_NUMBER}"
          // Print the current Jenkins build URL
          echo "Current Jenkins build URL: ${env.BUILD_URL}"
          // Print the current Jenkins workspace directory
          echo "Current Jenkins workspace directory: ${env.WORKSPACE}"
          // Print the current Jenkins executor number
          echo "Current Jenkins executor number: ${env.EXECUTOR_NUMBER}"
          // Print the current Jenkins label
          echo "Current Jenkins label: ${env.NODE_LABELS}"
  
          // Checkout the code from the repository
          checkout scm 
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
            def analysisResult = bat(script: 'C:/Users/mykol/.platformio/penv/Scripts/platformio.exe check --environment upesy_wroom > analysis.txt', returnStatus: true)
            if (analysisResult != 0) {
              echo "Static analysis report generation failed with exit code ${analysisResult}"
            } else {
              echo "Static analysis report generated successfully."
            }

            //create variable to store report generation result
            def reportDataGenerationResult = bat(script: 'python utility-scripts/parse-static-analysis-for-nested-data.py analysis.txt analisys.json', returnStatus: true)
            // Check result report data generation
            if (reportDataGenerationResult != 0) {
              echo "Report data generation failed with exit code ${reportDataGenerationResult}"
            } else {
              echo "Report data generated successfully."
            }
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
}