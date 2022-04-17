from unicodedata import name
from bs4 import BeautifulSoup
import subprocess

names = subprocess.check_output(['ls', 'blog']).decode("utf-8")
names = names.split('\n')[:-1]

for name in names:
    name = name.split('.')[0]
    figureNum = 1
    tableNum = 1
    eqNum = 1

    inputName = "../docs/blog/" + name + ".html"
    outputName = "../docs/blog/" + name + ".o"

    with open(inputName, 'r') as input:
        soup = BeautifulSoup(input, 'html.parser')
        ret = soup.find_all('span', class_='caption-number')

        for r in ret:
            value = r.string
            value = value.split(' ')[0]
            # Has to match the numfig_format set in conf.py
            if value == "Figure":
                r.string.replace_with('Figure ' + str(figureNum))
                figureNum += 1
            elif value == "Table":
                r.string.replace_with('Table ' + str(tableNum))
                tableNum += 1
            else:
                print("Wrong caption-number format: " + value)

        ret = soup.find_all('span', class_='eqno')

        for r in ret:
            value = r.next

            if value[0] == "(":
                r.next.replace_with('(' + str(eqNum) + ")")
                eqNum += 1
            else:
                print("Wrong equation number format: " + value)

        with open(outputName, 'w') as output:
            output.write(soup.prettify())

    proc = subprocess.Popen(['mv', outputName, inputName])
    proc.wait()
