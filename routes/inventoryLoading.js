const router = require('express').Router()
const Inventory = require('../model/inventory.model')

router.route('').get(async (req,res,next) => {
    const page = req.query.page || 1 // Page 
    const resPerPage = 5 // results per page
    await Inventory.find().sort({'createdAt':-1}).skip(page - resPerPage).limit(resPerPage)
    .exec((err, inventory) => {
        Inventory.countDocuments((err, count) => { // count how many page
          if (err) return next(err)
           res.send(inventory) // return data JSON
        })
      })
})

module.exports = router