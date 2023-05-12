import requests

def get_launches():
    api ="https://ll.thespacedevs.com/2.0.0/launch/upcoming/?fields=id(s),name,net"
    r = requests.get(api)
    r.encoding = 'UTF-8'
    json = r.json()
    s=";"
    try:
        with open("json.csv", 'w', encoding="utf-8") as f:
            f.write(str(json))
        #for event in json["eventLog"]:
         #   print(event["key"]+":  "+event["value"])
        i =0
        out=""
        if len(json['results'])>1:
            with open("launchids.csv", 'w') as f:
                f.write(str('id;name;net'+"\n"))
                for launch in json['results']:
                    out+=str(launch["launch_library_id"])+";"
                    #f.write(out+",")
                    out+=str(launch["name"])+";"
                    #f.write(out+",")
                    out+=str(launch["net"])+"\n"
                f.write(out)
        else:
            print("Unable to get new launch list data")
    except Exception as e:
        print(e)
        print('Failed to download future launch list')
        pass
