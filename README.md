# Emio.lab_empty

Build your own lab for the application [Emio Labs](https://docs-support.compliance-robotics.com/docs/next/Users/EmioLabs/). In this repository you will find a template and a good starting point to create your own lab from scratch.

## Description of the files

1. `lab_empty.md`: the markdown file to customize, and which will be displayed in the __Emio Labs__ application. 
2. `lab.json`: the json file for the application Emio Labs, with the title, description of the lab, and other info needed by the application:
    ```json
    {
        "name": "lab_empty", // the name of the lab folder
        "filename": "lab_empty.md", // the name of the markdown file
        "title": "Lab Empty", // the title,... 
        "description": "discover...", //... and description of the lab which will appear in the main table of contents of the Emio Labs application
    }
    ```
3. `lab_empty.py`: the python scene for __SOFA Robotics__, tipically a simulation of the robot Emio that you can launch from the Emio Labs application in exercises sections using buttons.  
4. `setLabName.sh`: the script to set the name of your lab. This will replace all occurences of `"empty"` with your chosen name. Usage is `setLabName.sh myName`. 
5. `requirements.txt`: the file to list the python packages required for your lab. See the README of the `modules/site-packages` directory. Or load the lab_empty in the Emio Labs application and read the "Install Additional Python Packages" section.

## Usage

Download this repository, use the script `setLabName.sh` to update all the files with the name of your lab. Add this lab to the application and you're good to go. 
You can follow [this documentation](https://docs-support.compliance-robotics.com/docs/next/Users/EmioLabs/create-your-lab/) to help you write the markdown file of your lab.
