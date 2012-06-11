
ft_viewer_html = '<table><tr><td>\
            <table border=1 style="height:450px; width:10px">\
        <tr id="ft_ref_danger" style="height:33%;background-color:red">\
          <td id="ft_ref_danger_label">[Danger Force Level]</td>\
        </tr><tr id="ft_ref_act" style="height:50%;background-color:green">\
          <td id="ft_ref_act_label">[Activity Force Level]</td>\
        </tr><tr id="ft_ref_null" style="height:100%;background-color:gray">\
         <td><div id="ft_readout"></div></td></tr></table>\
        </td><td><div id="ft_bar_wrapper">\
          <div id="ft_bar_value"></div></div></td></tr></table>'

$(function(){
$.widget("rfh.ft_viewer",{
    options:{
        clear:null
    },
    _create: function(){
        this.element.html(ft_viewer_html);
    },
    _destroy: function(){
        this.element.html('');
    }
});


});
