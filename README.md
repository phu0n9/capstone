# Schaeffler Inventory Checking Station

MERN stack website hosted on Heroku with ROS 

Access Link: https://schaeffler.herokuapp.com/

## Introduction

This is a set of API of an Ecommerce B2B websites like Alibaba or B2C websites like Tiki
  
## API Features
All the APIs share 4 main activities:
* Add new items
* Update items with respective ID
* Delete items with respective ID
* Search for items

Moreover, there are some more APIs that support some following functions:
* In **Order**, **Sale Invoice**, **Receiving Notes** and **DeliveryNote**, users can search by **Date**
* Revenue can be searched in **Sale Invoice**
* A list of **Inventory of all products** can be shown in the **InventoryControllerTest**

 
## Installation
* Open the project using **Intellij IDEA 2020**.
* Setup the JDK to version 14.

### Jetty Installation
**Link:** https://maven.apache.org/download.cgi

*```In Windows```*:
* Search for **Environment Variables** in start menu bar in Windows
* Edit **Path** by concatenating the path of the maven extracted folder

*```In Mac```*:
* Open **Terminal** and type *```~/.bash_profile```*
* Add 1 more line: *```export PATH="~/Downloads/apache-maven-3.6.3/bin:$PATH"```*

Check by the installation by typing *```mvn```* in the Terminal

### PostgreSQL Installation
Install PostgreSQL by this link:

*```Windows```* https://www.postgresql.org/download/windows/

*```Linux```* https://www.postgresql.org/download/macosx/

* After the installation, try to find the PostgreSQL files in the start menu bar and run **pgAdmin 4**
* The default password is **pgadmin** and it will ask you to change the password
* Set up account for **PostgreSQL** with the password in the class **AppConfig.java**.

### Compiler & Run
* Right click to the package Assignment 2 and chose **Terminal**.
* Type **```mvn jetty:run```** in the terminal

## Known bugs

All the bugs are handled successfully.

The order for running the test functions are:

* **CategoryControllerTest**
* **CustomerControllerTest**
* **StaffControllerTest**
* **ProductControllerTest**
* **ProviderControllerTest**
* **DeliveryControllerTest**
* **ReceivingNoteControllerTest**
* **SaleInvoiceControllerTest**

**```Attention:```** Should run delete method after other methods have been tested due to the test data in the Json files that I have inserted, so it would notify error because there are some mismatch between the inserted data and the test data

## Acknowledgement

* Mr Thanh's slides and answer from Team
* StackOverflow




