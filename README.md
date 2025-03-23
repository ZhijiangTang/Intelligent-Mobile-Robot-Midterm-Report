# Intelligent Mobile Robot Midterm Report
*ZhijiangTang @ HAIS, UCAS, 23/3/2025*

This is the code for the Intelligent Mobile Robot course midterm report. The detailed report is in [Report](Intelligent-Mobile-Robot-Midterm-Report.pdf).

### List
- Homework 1&2: [homework.ipynb](homework.ipynb)
- Homework 3: [homework3.py](homework3.py)
    - run it:
        ```bash
        ros2 bag play ./data/output.db3
        # new termunal
        python3 homework3.py
        ```
- Raw data: [data](data)
- Midterm Report: [Report](Intelligent-Mobile-Robot-Midterm-Report.pdf)
- Midterm homework requirements: [中期作业.pdf](中期作业.pdf)
- A demo for homework 3: [Homwork3 Demo.mp4](Homwork3%20Demo.mp4)


### Help
```bash
ros2 bag play ./data/output.db3
ros2 topic echo /scan --no-arr
```