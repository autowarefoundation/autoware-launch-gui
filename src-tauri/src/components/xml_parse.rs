use serde_json::json;
use std::io::BufReader;
use std::{fs::File, path::Path};
use tauri::Wry;
use xml::reader::{EventReader, XmlEvent};

#[derive(Debug)]
pub struct Element {
    name: String,
    attributes: Vec<(String, String)>,
    children: Vec<Element>,
}

impl Element {
    pub fn to_json(&self) -> serde_json::Value {
        json!({
            "name": self.name,
            "attributes": self.attributes,
            "children": self.children.iter().map(|child| child.to_json()).collect::<Vec<_>>()
        })
    }
}

#[tauri::command]
pub fn parse_and_send_xml(window: tauri::Window<Wry>, path: String, calibration_tool: bool) {
    let path = Path::new(&path);
    // Open the XML file
    let file = File::open(path).ok().expect("Unable to open file");
    let file = BufReader::new(file);

    let parser = EventReader::new(file);
    let mut elements: Vec<Element> = Vec::new();
    let mut stack: Vec<Element> = Vec::new();

    for e in parser {
        match e {
            Ok(XmlEvent::StartElement {
                name, attributes, ..
            }) => {
                let element = Element {
                    name: name.local_name,
                    attributes: attributes
                        .into_iter()
                        .map(|attr| (attr.name.local_name, attr.value))
                        .collect(),
                    children: Vec::new(),
                };
                stack.push(element);
            }
            Ok(XmlEvent::EndElement { name }) => {
                if let Some(element) = stack.pop() {
                    if element.name == name.local_name {
                        if let Some(parent) = stack.last_mut() {
                            parent.children.push(element);
                        } else {
                            elements.push(element);
                        }
                    }
                }
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
            _ => {}
        }
    }

    let elements_json = json!({
        "elements": elements.iter().map(|element| element.to_json()).collect::<Vec<_>>()
    });

    let window2 = window.clone();

    if calibration_tool {
        window2
            .emit("receiveTreeCalibration", Some(elements_json.to_string()))
            .expect("Failed to emit tree");
    } else {
        window
            .emit("receiveTree", Some(elements_json.to_string()))
            .expect("Failed to emit tree");
    }
}
