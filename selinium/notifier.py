import time
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.options import Options

SEPARATOR = "="*30

options = Options()
options.add_experimental_option("detach", True)
options.add_argument('--remote-debugging-port=9222')
browser = webdriver.Chrome()  

browser.get("http://bit.ly/35KqkKb")
input('Wait till you have logged in and see the screen. Press \'y\': ')

# Initially do 2 iterations of this. Then run infinite loop
x = [True]*2+[False]
model_number = 0
count = 0
while True:
        
    try:
        browser.refresh()
        time.sleep(10)
        browser.execute_script("window.scrollTo(0,document.body.scrollHeight)")
        time.sleep(10)
        try:
            btn = browser.find_element(By.CSS_SELECTOR, '.awsui_button_vjswe_1wo5s_105.awsui_variant-normal_vjswe_1wo5s_156.awsui_disabled_vjswe_1wo5s_241')
        except:
            btn = None
        
        print(SEPARATOR)
        print(f'Submission {count}: ')
        
        # if condition to check if we could get the disabled button 
        if not btn:
            # Code to submit model 
            count += 1 
            # awsui_button_vjswe_r2ttg_101 awsui_variant-normal_vjswe_r2ttg_126
            time.sleep(10)
            browser.execute_script("window.scrollTo(0,0)")
            btn_race_again1 = browser.find_elements(By.CSS_SELECTOR, '.awsui_button_vjswe_1wo5s_105.awsui_variant-primary_vjswe_1wo5s_249')
            for bt in btn_race_again1 :
                if(bt.text == "Race again"):
                    bt.click()
                    break
            # btn_race_again = browser.find_elements(By.CSS_SELECTOR, '.awsui_button_vjswe_1wo5s_105.awsui_variant-primary_vjswe_1wo5s_249')[1]
            # btn_race_again.click()
            time.sleep(10)

            
            browser.execute_script("window.scrollTo(0,document.body.scrollHeight)")
            time.sleep(10)
            btn_choose_model = browser.find_elements(By.CSS_SELECTOR, '.awsui_button-trigger_18eso_7pgwp_101.awsui_has-caret_18eso_7pgwp_168')
            for bt in btn_choose_model :
                if(bt.text == "Choose a model"):
                    bt.click()
                    break

            # btn_choose_model.click() 
            time.sleep(5)
            # Here need to see how to submit the model. 
            # For now, we will do a manual submission. 
            option = browser.find_elements(By.CSS_SELECTOR, '.awsui_options-list_19gcf_wdxz2_97')
            print(len(option))
            c = 0
            for bt in option :
                print("Option " + str(c))
                c += 1
                print(bt.text)
            options = browser.find_elements(By.CSS_SELECTOR, '.awsui_options-list_19gcf_wdxz2_97')[1]

            #model_to_submit = random.choice(["Stable-model-1-clone-clone-mt-mt-clone-2"]*1+["Stable-model-1-clone-clone-mt-mt-clone-2"]*15)
            model1 = 'model-2-clone-mt-mt-clone-1-mt-mt-clone'
            model2 = 'model-1-clone-mt-cnhg-fun-clone-clone'
            model3 = 'model-2-clone-mt-mt-clone-1-mt-chng-clone'
            model4 = 'model-2-clone-mt-mt-clone-1-mt-chng'
            model5 = 'model-1-clone-mt-cnhg-fun-clone'
            models_to_submit = [model1, model2,model3,model4,model5]
            model_to_submit = models_to_submit[model_number%len(models_to_submit)]
            selected_option = browser.find_element(By.XPATH,f'//span[contains(text(),{model_to_submit})]')
            browser.execute_script("arguments[0].click();", selected_option)

            print("Options: ",options.text)
            list_items = options.find_elements(By.TAG_NAME, 'li')
            print("Items list: ", list_items)
            for list_item in list_items:
                model_name = list_item.text.split('\n')[0]
                print(model_name, model_name==model_to_submit)
                #text_of_model = list_item.find_element(By.CSS_SELECTOR, '.awsui_screenreader-content_15o6u_htcil_219').text 
                if model_name==model_to_submit:
                    print(f'Submitting, {model_name}')
                    print(list_item)
                    list_item.click()    
                    break
            
            time.sleep(5)
            submit_button = browser.find_elements(By.CSS_SELECTOR, '.awsui_button_vjswe_1wo5s_105.awsui_variant-primary_vjswe_1wo5s_249')
            for bt in submit_button :
                if(bt.text == "Enter race"):
                    bt.click()
                    break
            # print(submit_button.)
            # submit_button.click()
            # input('Enter something once the model is submitted.')
            print('Submission Successful.')
            model_number += 1
            time.sleep(20)
            browser.back()
            browser.back()
            for i in range(6):
                browser.refresh()
                time.sleep(10)
        else:
            # Code to do nothing and refresh model
            print('Will have to try again...currently something else is submitting')
        

        for i in range(6):
            browser.refresh()
            time.sleep(10)
    
    except Exception as e:
        print(e)
        if str(e).find('no such window')!=-1:
            print('Window not present.')
            break

